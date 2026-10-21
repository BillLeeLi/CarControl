"""PD/PI 的数值、饱和、变化率限制、时间与复位边界。"""

from dataclasses import replace
import math

import pytest

from process_node.control import ControlInput, ControlReference, MotionFeedback
from process_node.pid_controller import PIDConfig, PIDController
from test_control import feedback


def inputs(t, heading=0.0, target=0.2, measured=0.0, reference_id='cruise'):
    sample = feedback()
    sample = replace(sample, state=replace(sample.state, stamp_ns=round(t * 1e9),
                                           speed_mps=measured))
    return ControlInput(ControlReference(round(t * 1e9), heading, target,
                                         reference_id=reference_id), sample)


def config(**changes):
    """取消滤波/斜坡对基础公式的影响；各自的限制由独立用例检查。"""
    base = PIDConfig(steer_sign=1, steer_kp=10.0, steer_kd=2.0,
                     steer_derivative_tau_s=0.0, steer_rate_limit=1e6,
                     speed_feedforward_gain=0.0, speed_kp=0.0, speed_ki=0.0,
                     speed_rate_limit=1e6, target_accel_mps2=100.0,
                     target_decel_mps2=100.0)
    return replace(base, **changes)


def test_pd_formula_uses_message_dt_and_calibrated_center():
    controller = PIDController(config(steer_center=20))
    warmup = controller.compute(inputs(1, heading=0.1))
    assert (warmup.steer, warmup.speed, warmup.status) == (20, 0, 'CONTROL_WARMUP')
    command = controller.compute(inputs(1.1, heading=0.2))
    assert command.steer == 24  # 中位 20 + P 项 2 + D 项 2
    assert command.diagnostics.heading_rate_rps == pytest.approx(1)


def test_pd_derivative_filter_and_target_jump_reset():
    controller = PIDController(config(steer_derivative_tau_s=0.2))
    controller.compute(inputs(1))
    command = controller.compute(inputs(1.1, heading=0.1))
    assert command.diagnostics.heading_rate_rps == pytest.approx(1 - math.exp(-0.5))
    jumped = controller.compute(inputs(1.2, heading=math.radians(40)))
    assert jumped.diagnostics.derivative_reset
    assert jumped.diagnostics.steer_d == 0


def test_pi_increases_effort_when_slow_and_coasts_when_faster_than_target():
    controller = PIDController(config(speed_kp=100.0))
    controller.compute(inputs(1))
    slow = controller.compute(inputs(1.1, measured=0))
    closer = controller.compute(inputs(1.2, measured=0.1))
    fast = controller.compute(inputs(1.3, measured=0.3))
    assert (slow.speed, closer.speed, fast.speed) == (-20, -10, 0)
    assert fast.diagnostics.speed_saturated  # 下界是零动力，不通过倒车命令制动


def test_speed_mapping_respects_nonzero_stop_and_forward_sign():
    controller = PIDController(config(stop_speed=10, speed_sign=1, speed_kp=100.0))
    controller.compute(inputs(1))
    assert controller.compute(inputs(1.1)).speed == 30
    assert controller.compute(inputs(1.2, target=0)).speed == 10


def test_integral_accumulates_with_dt_and_has_its_own_bound():
    controller = PIDController(config(speed_ki=10.0, speed_integral_limit=0.3))
    controller.compute(inputs(1))
    assert controller.compute(inputs(1.1)).diagnostics.speed_i == pytest.approx(0.2)
    assert controller.compute(inputs(1.2)).diagnostics.speed_i == pytest.approx(0.3)
    assert controller.compute(inputs(1.3)).diagnostics.speed_i == pytest.approx(0.3)


def test_output_saturation_does_not_wind_up_and_can_recover():
    controller = PIDController(config(speed_kp=1000.0, speed_ki=100.0,
                                      speed_effort_limit=10.0))
    controller.compute(inputs(1))
    for i in range(1, 31):
        command = controller.compute(inputs(1 + i * 0.1))
        assert command.speed == -10
        assert command.diagnostics.integral_frozen
        assert command.diagnostics.speed_i == 0
    recovered = controller.compute(inputs(4.1, measured=0.195))
    assert -10 < recovered.speed < 0
    assert not recovered.diagnostics.integral_frozen


def test_slew_limit_also_prevents_integral_windup():
    controller = PIDController(config(speed_feedforward_gain=100.0, speed_ki=100.0,
                                      speed_rate_limit=10.0))
    controller.compute(inputs(1))
    command = controller.compute(inputs(1.1))
    assert command.speed == -1
    assert command.diagnostics.speed_rate_limited
    assert command.diagnostics.integral_frozen
    assert command.diagnostics.speed_i == 0


def test_steering_saturation_and_rate_limit_are_both_applied():
    controller = PIDController(config(steer_kp=1000.0, steer_kd=0.0,
                                      steer_limit=100, steer_rate_limit=100.0))
    controller.compute(inputs(1))
    left = controller.compute(inputs(1.1, heading=0.5))
    assert left.steer == 10
    assert left.diagnostics.steer_saturated and left.diagnostics.steer_rate_limited
    right = controller.compute(inputs(1.2, heading=-0.5))
    assert right.steer == 0


def test_internal_target_ramp_and_zero_speed_stop():
    controller = PIDController(PIDConfig())
    controller.compute(inputs(1))
    command = controller.compute(inputs(1.1))
    assert command.diagnostics.target_speed_mps == pytest.approx(0.02)
    assert command.diagnostics.requested_speed_mps == 0.2
    stopped = controller.compute(inputs(1.2, target=0))
    assert stopped.speed == 0
    assert stopped.steer == 0
    assert stopped.status == 'STOP_REQUESTED'
    assert controller.compute(inputs(1.3)).status == 'CONTROL_WARMUP'
    assert controller.compute(inputs(1.4)).diagnostics.target_speed_mps == pytest.approx(0.02)


def test_positive_target_reduction_uses_deceleration_limit():
    controller = PIDController(config(target_decel_mps2=0.1))
    controller.compute(inputs(1, target=0.3))
    controller.compute(inputs(1.1, target=0.3))
    reduced = controller.compute(inputs(1.2, target=0.1))
    assert reduced.diagnostics.target_speed_mps == pytest.approx(0.29)


def test_reference_switch_resets_integral_and_derivative_but_limits_output_change():
    controller = PIDController(config(speed_ki=10.0))
    controller.compute(inputs(1))
    for i in range(1, 11):
        previous = controller.compute(inputs(1 + i * 0.1))
    assert previous.diagnostics.speed_i == pytest.approx(2)
    switched = controller.compute(inputs(2.1, heading=0.1, reference_id='new_behavior'))
    assert switched.diagnostics.speed_i == pytest.approx(0.2)
    assert switched.diagnostics.derivative_reset
    assert switched.diagnostics.steer_d == 0


def test_short_intervals_accumulate_without_assuming_a_fixed_frame_rate():
    controller = PIDController(config(speed_ki=10.0))
    controller.compute(inputs(1))
    assert controller.compute(inputs(1.005)).status == 'CONTROL_WARMUP'
    command = controller.compute(inputs(1.01))
    assert command.diagnostics.dt_s == pytest.approx(0.01)
    assert command.diagnostics.speed_i == pytest.approx(0.02)


@pytest.mark.parametrize('t,status', [(1.1, 'CONTROL_TIME_INVALID'),
                                     (1.0, 'CONTROL_TIME_INVALID'),
                                     (2.0, 'CONTROL_TIME_GAP')])
def test_duplicate_backward_or_large_time_jump_stops_and_restarts(t, status):
    controller = PIDController(config(speed_kp=100.0))
    controller.compute(inputs(1))
    assert controller.compute(inputs(1.1)).speed != 0
    stopped = controller.compute(inputs(t))
    assert stopped.status == status
    assert stopped.speed == 0
    assert controller.compute(inputs(t + 0.1)).status == 'CONTROL_WARMUP'


def test_feedback_loss_and_protection_stop_clear_integral_and_derivative():
    controller = PIDController(config(speed_ki=10.0))
    controller.compute(inputs(1, heading=0.1))
    controller.compute(inputs(1.1, heading=0.2))
    missing = replace(inputs(1.2), feedback=MotionFeedback(None, 'STALE_ODOM', '超时'))
    assert controller.compute(missing).speed == 0
    assert controller.compute(inputs(1.3)).status == 'CONTROL_WARMUP'
    restarted = controller.compute(inputs(1.4, target=0.2, measured=0.2))
    assert restarted.diagnostics.speed_i == 0
    assert restarted.diagnostics.steer_d == 0
    assert controller.stop('TEST_STOP', '保护覆盖').speed == 0
    assert controller.compute(inputs(1.5)).status == 'CONTROL_WARMUP'


@pytest.mark.parametrize('target', [None, -0.1, 0.6, float('nan'), float('inf')])
def test_invalid_target_speed_never_produces_motion(target):
    command = PIDController().compute(inputs(1, target=target))
    assert command.status == 'INVALID_REFERENCE'
    assert command.speed == 0


def test_nonfinite_velocity_feedback_is_rejected():
    command = PIDController().compute(inputs(1, measured=float('nan')))
    assert command.status == 'INVALID_FEEDBACK'


def test_same_message_sequence_is_deterministic_and_integral_scales_with_time():
    def run(dt):
        controller = PIDController(config(speed_ki=10.0))
        controller.compute(inputs(1))
        return [controller.compute(inputs(1 + dt * i)) for i in range(1, round(1 / dt) + 1)]
    assert run(0.1) == run(0.1)
    assert run(0.1)[-1].diagnostics.speed_i == pytest.approx(2)
    assert run(0.05)[-1].diagnostics.speed_i == pytest.approx(2)


def test_negative_feedback_converges_in_an_explicitly_simplified_plant():
    """人为一阶速度/转向模型只检验反馈方向与数值收敛，不代表真实小车。"""
    controller = PIDController()
    velocity, yaw = 0.0, 0.0
    for i in range(401):
        command = controller.compute(inputs(1 + 0.05 * i, heading=0.1 - yaw,
                                             measured=velocity))
        # 假定每个前进动力单位对应 0.005 m/s，速度时间常数 0.4 s。
        velocity += 0.05 * (0.005 * (-command.speed) - velocity) / 0.4
        # 假定负 steer 产生正偏航，系数 0.004 rad/s/控制单位。
        yaw += 0.05 * (-0.004 * command.steer)
    assert velocity == pytest.approx(0.2, abs=0.01)
    assert yaw == pytest.approx(0.1, abs=0.01)


@pytest.mark.parametrize('changes', [
    {'steer_sign': 0}, {'speed_sign': 2}, {'speed_kp': -1}, {'speed_ki': float('nan')},
    {'speed_effort_limit': 40000.0}, {'control_min_dt_s': 0.6},
    {'steer_jump_reset_deg': 181.0}, {'speed_rate_limit': 0.0},
])
def test_invalid_configurations_are_rejected(changes):
    with pytest.raises(ValueError):
        replace(PIDConfig(), **changes)

"""运动反馈的单位、符号、时间关联及失效处理。"""

from dataclasses import replace
import math
from types import SimpleNamespace as NS

import pytest

from process_node.motion import MotionConfig, MotionEstimator


def odom(t, x=0.0, y=0.0, yaw=0.0, frame='odom'):
    """只提供位置/姿态；twist 故意填入错误速度，验证实现不会依赖它。"""
    stamp_ns = round(t * 1e9)
    return NS(
        header=NS(stamp=NS(sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9), frame_id=frame),
        child_frame_id='base_link',
        pose=NS(pose=NS(position=NS(x=x, y=y), orientation=NS(
            x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))),
        twist=NS(twist=NS(linear=NS(x=999.0, y=999.0))),
    )


def test_stationary_forward_reverse_and_distance_have_physical_units():
    estimator = MotionEstimator(MotionConfig(filter_tau_s=0.0))
    assert not estimator.update(odom(1), received_at=10).valid
    assert estimator.update(odom(1.1), received_at=10.1).state.speed_mps == 0
    forward = estimator.update(odom(1.2, x=0.05), received_at=10.2)
    assert forward.valid
    assert forward.state.speed_mps == pytest.approx(0.5)
    reverse = estimator.update(odom(1.3, x=0.03), received_at=10.3).state
    assert reverse.speed_mps == pytest.approx(-0.2)
    assert reverse.distance_m == pytest.approx(0.07)
    assert reverse.signed_distance_m == pytest.approx(0.03)


def test_speed_projection_and_local_position_follow_starting_heading():
    estimator = MotionEstimator()
    estimator.update(odom(1, x=5, y=3, yaw=math.pi / 2), received_at=10)
    state = estimator.update(odom(1.1, x=5, y=3.1, yaw=math.pi / 2), received_at=10.1).state
    assert state.speed_mps == pytest.approx(1)
    assert state.local_x_m == pytest.approx(0.1)
    assert state.local_y_m == pytest.approx(0)
    assert state.x_m == 5


def test_heading_unwrap_and_yaw_rate_cross_pi_without_false_jump():
    estimator = MotionEstimator(MotionConfig(filter_tau_s=0))
    estimator.update(odom(1, yaw=math.radians(179)), received_at=10)
    state = estimator.update(odom(1.1, yaw=math.radians(-179)), received_at=10.1).state
    assert state.heading_change_rad == pytest.approx(math.radians(2))
    assert state.yaw_unwrapped_rad == pytest.approx(math.radians(181))
    assert state.yaw_rate_rps == pytest.approx(math.radians(20))


def test_filter_uses_message_dt_and_does_not_depend_on_playback_speed():
    states = []
    for playback_speed in (1, 3):
        estimator = MotionEstimator(MotionConfig(filter_tau_s=0.2))
        for i, x in enumerate((0.0, 0.1, 0.1)):
            feedback = estimator.update(odom(1 + 0.1 * i, x=x),
                                        received_at=10 + 0.1 * i / playback_speed)
        states.append(feedback.state)
    assert states[0] == states[1]
    assert states[0].raw_speed_mps == 0
    assert states[0].speed_mps == pytest.approx(math.exp(-0.1 / 0.2))


def test_tiny_intervals_accumulate_and_duplicates_do_not_refresh_liveness():
    estimator = MotionEstimator()
    estimator.update(odom(1), received_at=10)
    assert not estimator.update(odom(1.005, x=0.005), received_at=10.005).valid
    feedback = estimator.update(odom(1.01, x=0.01), received_at=10.01)
    assert feedback.valid
    assert feedback.state.raw_speed_mps == pytest.approx(1)
    duplicate = estimator.update(odom(1.01, x=100), received_at=10.2)
    assert not duplicate.valid
    assert estimator.last_received == 10.01
    recovered = estimator.update(odom(1.02, x=0.02), received_at=10.21)
    assert recovered.valid
    assert recovered.state.raw_speed_mps == pytest.approx(1)


@pytest.mark.parametrize('next_time,x,yaw,status', [
    (0.5, 0.0, 0.0, 'ODOM_TIME_REWIND'),
    (2.0, 0.0, 0.0, 'ODOM_TIME_GAP'),
    (1.2, 100.0, 0.0, 'ODOM_POSE_JUMP'),
    (1.2, 0.0, 2.0, 'ODOM_POSE_JUMP'),
])
def test_discontinuities_clear_old_state_and_require_a_new_difference(next_time, x, yaw, status):
    estimator = MotionEstimator()
    estimator.update(odom(1), received_at=10)
    estimator.update(odom(1.1, x=0.1), received_at=10.1)
    previous_generation = estimator.generation
    feedback = estimator.update(odom(next_time, x=x, yaw=yaw), received_at=10.2)
    assert feedback.status == status
    assert feedback.state is None
    assert estimator.generation > previous_generation
    recovered = estimator.update(odom(next_time + 0.1, x=x, yaw=yaw), received_at=10.3)
    assert recovered.valid
    assert recovered.state.distance_m == 0
    assert recovered.state.heading_change_rad == 0


@pytest.mark.parametrize('bad_kind', ['nan', 'zero_quaternion', 'frame', 'child_frame', 'stamp'])
def test_invalid_messages_invalidate_old_feedback(bad_kind):
    estimator = MotionEstimator()
    estimator.update(odom(1), received_at=10)
    estimator.update(odom(1.1), received_at=10.1)
    bad = odom(1.2)
    if bad_kind == 'nan':
        bad.pose.pose.position.x = float('nan')
    elif bad_kind == 'zero_quaternion':
        bad.pose.pose.orientation.w = 0
    elif bad_kind == 'frame':
        bad.header.frame_id = 'map'
    elif bad_kind == 'child_frame':
        bad.child_frame_id = 'laser_link'
    else:
        bad.header.stamp.nanosec = 10**9
    feedback = estimator.update(bad, received_at=10.2)
    assert feedback.status == 'INVALID_ODOM'
    assert feedback.state is None
    assert not estimator.feedback(round(1.1e9), now=10.2).valid


def test_alignment_uses_history_not_just_latest_and_refuses_extrapolation():
    estimator = MotionEstimator()
    for i in range(6):
        estimator.update(odom(1 + 0.1 * i, x=0.1 * i), received_at=10 + 0.1 * i)
    feedback = estimator.feedback(round(1.22e9), now=10.5)
    assert feedback.valid
    assert feedback.state.stamp_ns == round(1.2e9)
    assert feedback.skew_s == pytest.approx(-0.02)
    assert estimator.feedback(round(2e9), now=10.5).status == 'UNALIGNED_ODOM'
    assert estimator.feedback(round(1.2e9), now=11.1).status == 'STALE_ODOM'


def test_feedback_history_is_bounded_by_message_time():
    estimator = MotionEstimator(MotionConfig(history_s=0.7))
    for i in range(30):
        estimator.update(odom(1 + 0.1 * i), received_at=10 + 0.1 * i)
    assert estimator.feedback(round(1.2e9), now=12.9).status == 'UNALIGNED_ODOM'
    assert estimator.feedback(round(3.8e9), now=12.9).valid


@pytest.mark.parametrize('changes', [
    {'min_dt_s': 0}, {'max_dt_s': 0.001}, {'filter_tau_s': -1},
    {'timeout_s': float('nan')}, {'history_s': 0.01}, {'frame_id': ''},
])
def test_invalid_configurations_are_rejected(changes):
    with pytest.raises(ValueError):
        replace(MotionConfig(), **changes)

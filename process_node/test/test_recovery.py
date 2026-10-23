"""局部恢复的安全门控、真实墙段短弧及预算；无 ROS 依赖。"""

from dataclasses import replace
import math
from types import SimpleNamespace

import numpy as np
import pytest

from process_node.corridor import CorridorEstimate
from process_node.perception import PlannerConfig
from process_node.recovery import LocalRecoveryBehavior, RecoveryConfig
from process_node.turning import TurnConfig, TurnResult, check_swept_path
from test_corridor import feedback as pose_feedback
from test_exits import corridor, road


IDLE = TurnResult('CRUISE', 'OK', 'cruise')
SOFT = TurnResult('FAULT', 'TURN_FAULT', 'lost path', active=True, recoverable=True)
GOAL = ((2.3, -1.1), -math.pi/2, 7)
LOST = CorridorEstimate('CORRIDOR_LOST', 'lost walls')


def feedback(*args, speed=0., **kwargs):
    result = pose_feedback(*args, **kwargs)
    return replace(result, state=SimpleNamespace(**vars(result.state), speed_mps=speed))


def behavior(**kwargs):
    return LocalRecoveryBehavior(RecoveryConfig(**kwargs), PlannerConfig(), TurnConfig())


def update(b, t, *, data=None, motion=None, walls=None, angle=None,
           status='CRUISE_BLOCKED', turn=IDLE, goal=None):
    return b.update(road(branch=None) if data is None else data,
                    feedback() if motion is None else motion, round(t*1e9),
                    corridor() if walls is None else walls, angle, status, turn, goal)


def test_normal_cruise_is_unchanged_and_initial_acquiring_does_not_start_recovery():
    b = behavior()
    assert not update(b, 1., angle=0., status='OK').active
    assert not update(b, 1.1, walls=LOST, status='CORRIDOR_ACQUIRING').active


def test_stops_then_confirms_a_safe_curve_toward_the_corridor_center():
    b = behavior()
    walls = replace(corridor(), lateral_error_m=.3)
    first = update(b, 1., walls=walls)
    assert first.phase == 'WAIT' and first.heading_rad is None
    for t in (1.1, 1.2):
        result = update(b, t, walls=walls)
        assert result.phase == 'CONFIRM' and result.heading_rad is None
    result = update(b, 1.3, walls=walls)
    assert result.phase == 'MOVE' and result.status == 'OK'
    assert result.heading_rad > 0 and result.speed_mps == .06
    assert check_swept_path(road(branch=None), result.swept_body, b.planner)[0]


def test_recorded_motion_must_slow_down_before_recovery_can_drive():
    b = behavior()
    moving = feedback(speed=.3)
    for i in range(8):
        result = update(b, 1+i*.1, motion=moving)
        assert result.heading_rad is None
    assert result.status == 'RECOVERY_WAITING'
    for i in range(4):
        result = update(b, 1.8+i*.1)
    assert result.phase == 'MOVE'


def test_transient_obstruction_clearance_cancels_wait_without_stealing_normal_control():
    b = behavior()
    update(b, 1.)
    result = update(b, 1.1, angle=0., status='OK')
    assert not result.active and result.phase == 'IDLE'


def test_unknown_region_cannot_be_used_as_a_recovery_escape():
    b = behavior()
    data = road(branch=None)
    data.bin_ranges[:] = np.nan
    for i in range(7):
        result = update(b, 1+i*.1, data=data)
        assert result.heading_rad is None
    assert result.status == 'RECOVERY_BLOCKED'


def test_real_obstacle_blocks_every_recovery_curve():
    b = behavior()
    from test_exits import raycast
    data = raycast([((-4., 1.1), (4., 1.1)), ((-4., -1.1), (4., -1.1)),
                    ((.30, -1.1), (.30, 1.1))])
    for i in range(5):
        result = update(b, 1+i*.1, data=data)
    assert result.status == 'RECOVERY_BLOCKED' and result.heading_rad is None


def test_no_road_guide_does_not_turn_toward_pure_no_return_space():
    b = behavior()
    for i in range(5):
        result = update(b, 1+i*.1, walls=LOST)
    assert result.status == 'RECOVERY_UNAVAILABLE' and result.heading_rad is None
    assert not result.active and b._attempts == 0


def test_recent_road_guide_expires_in_time_and_distance():
    for travel in (False, True):
        b = behavior(guide_timeout_s=.4)
        update(b, 1., angle=0., status='OK')
        update(b, 1.1, walls=LOST)
        if travel:
            result = update(b, 1.2, motion=feedback(x=.4), walls=LOST)
        else:
            for t in (1.2, 1.3, 1.4, 1.5):
                result = update(b, t, walls=LOST)
        assert result.status in ('RECOVERY_NO_GUIDE', 'RECOVERY_UNAVAILABLE')
        assert result.heading_rad is None


def test_expired_guide_before_first_motion_cancels_recovery_instead_of_faulting_future_roads():
    b = behavior(guide_timeout_s=.25)
    update(b, 1., angle=0., status='OK')
    update(b, 1.1, walls=LOST)
    update(b, 1.2, walls=LOST)
    result = update(b, 1.3, walls=LOST)
    assert not result.active and result.status == 'RECOVERY_UNAVAILABLE'
    result = update(b, 1.4, angle=0., status='OK')
    assert not result.active


def test_selected_exit_intent_cannot_be_replaced_by_open_straight_corridor():
    b = behavior()
    for i in range(5):
        result = update(b, 1+i*.1, data=road(), angle=0., status='OK', turn=SOFT, goal=GOAL)
    assert not result.handoff
    assert result.phase == 'MOVE' and result.curvature_per_m < 0
    assert result.cause == 'lost path'


def test_a_reliable_new_road_clears_soft_turn_fault_after_whole_body_crosses_gate():
    b = behavior()
    pose = (2.3, -1.6, -math.pi/2)
    walls = replace(corridor(), heading_rad=0., lateral_error_m=0., width_m=1.8)
    for i in range(4):
        result = update(b, 1+i*.1, data=road(pose=pose), motion=feedback(*pose),
                        walls=walls, angle=0., status='OK', turn=SOFT, goal=GOAL)
        if i < 3:
            assert result.heading_rad is None and not result.handoff
    assert result.handoff and result.status == 'RECOVERY_COMPLETE'
    assert not b.active


def test_handoff_rejects_a_car_whose_rear_has_not_crossed_the_exit():
    b = behavior()
    pose = (2.3, -1.3, -math.pi/2)
    for i in range(5):
        result = update(b, 1+i*.1, data=road(pose=pose), motion=feedback(*pose),
                        walls=corridor(), angle=0., status='OK', turn=SOFT, goal=GOAL)
    assert not result.handoff


def test_hard_turn_fault_is_never_overridden_even_on_a_clear_road():
    b = behavior()
    hard = replace(SOFT, recoverable=False)
    for i in range(5):
        result = update(b, 1+i*.1, angle=0., status='OK', turn=hard, goal=GOAL)
    assert not result.active and result.heading_rad is None


def test_input_interruption_requires_confirmation_again_and_never_reuses_old_output():
    b = behavior()
    for i in range(4):
        result = update(b, 1+i*.1)
    assert result.phase == 'MOVE'
    result = update(b, 1.4, motion=feedback(valid=False))
    assert result.heading_rad is None and result.phase == 'WAIT'
    for t in (1.5, 1.6):
        assert update(b, t).heading_rad is None
    assert update(b, 1.7).phase == 'MOVE'


@pytest.mark.parametrize('time', [1., .9, 2.1])
def test_duplicate_rewind_or_long_gap_latches_recovery_fault(time):
    b = behavior()
    update(b, 1.)
    result = update(b, time)
    assert result.status == 'RECOVERY_FAULT'
    assert update(b, 2.2, angle=0., status='OK').status == 'RECOVERY_FAULT'


def test_pose_reset_interrupt_is_hard_and_preserves_first_failure():
    b = behavior()
    update(b, 1.)
    b.interrupt('odom pose jump', hard=True)
    b.interrupt('later timeout')
    assert update(b, 1.1).reason == 'odom pose jump'


def test_real_receive_gap_is_checked_even_when_message_time_barely_moves():
    b = behavior()
    b.update(road(), feedback(), 10**9, corridor(), None, 'CRUISE_BLOCKED', IDLE,
             received_at=100.)
    result = b.update(road(), feedback(), 1100000000, corridor(), None,
                      'CRUISE_BLOCKED', IDLE, received_at=102.)
    assert result.status == 'RECOVERY_FAULT' and result.heading_rad is None


def test_invalid_pose_feedback_is_not_treated_as_a_transient_receive_timeout():
    b = behavior()
    update(b, 1.)
    bad = replace(feedback(), status='ODOM_POSE_JUMP')
    result = update(b, 1.1, motion=bad)
    assert result.status == 'RECOVERY_FAULT' and result.heading_rad is None


def test_obstacle_wait_does_not_consume_executing_progress_window():
    b = behavior(progress_timeout_s=.3)
    for i in range(4):
        assert update(b, 1+i*.1).phase != 'FAULT'
    blocked = road(branch=None)
    blocked.bin_ranges[:] = np.nan
    for i in range(8):
        assert update(b, 1.4+i*.1, data=blocked).status == 'RECOVERY_BLOCKED'
    for i in range(3):
        result = update(b, 2.2+i*.1)
    assert result.phase == 'MOVE'


@pytest.mark.parametrize('budget', ['time', 'travel', 'progress', 'attempts'])
def test_recovery_budget_is_bounded_and_cannot_be_reset_by_waiting(budget):
    b = behavior(max_duration_s=.35) if budget == 'time' else behavior(
        max_travel_m=.10) if budget == 'travel' else behavior(
        progress_timeout_s=.2) if budget == 'progress' else behavior(max_attempts=2)
    if budget == 'attempts':
        for i in range(3):
            result = update(b, 1+i*.2)
            if result.phase == 'FAULT':
                break
            update(b, 1.1+i*.2, angle=0., status='OK')
    else:
        for i in range(8):
            # 两次小位移累计超过门槛；不能只用离起点的净位移。
            x = (.06 if i % 2 else 0.) if budget == 'travel' else 0.
            result = update(b, 1+i*.1, motion=feedback(x=x))
            if result.phase == 'FAULT':
                break
    assert result.status == 'RECOVERY_FAULT' and result.heading_rad is None


def test_normal_stability_refreshes_retry_budget():
    b = behavior(max_attempts=1)
    update(b, 1.)
    for i in range(12):
        update(b, 1.1+i*.1, angle=0., status='OK')
    assert update(b, 2.3).phase == 'WAIT'


def test_kinematic_recovery_corrects_lateral_error_and_hands_back_to_cruise():
    # 只验证局部几何闭环；位姿按期望曲率积分，不声称底层车辆会精确执行曲率。
    b = behavior(max_duration_s=35., max_travel_m=1.5)
    x, y, yaw = 0., .20, 0.
    speed, phases = 0., set()
    for i in range(360):
        pose = (x, y, yaw)
        ready = abs(y) < .06 and abs(yaw) < math.radians(10)
        result = update(b, 1+i*.1, data=road(branch=None, pose=pose),
                        motion=feedback(*pose, speed=speed), walls=corridor(pose),
                        angle=0. if ready else None, status='OK' if ready else 'CRUISE_BLOCKED')
        phases.add(result.phase)
        assert result.phase != 'FAULT', result.reason
        if result.handoff:
            break
        speed = result.speed_mps if result.heading_rad is not None else 0.
        ds, k = speed*.1, result.curvature_per_m
        x += ds*math.cos(yaw+k*ds/2)
        y += ds*math.sin(yaw+k*ds/2)
        yaw += k*ds
    assert result.handoff
    assert {'WAIT', 'CONFIRM', 'MOVE', 'HANDOFF'} <= phases
    assert abs(y) < .06


@pytest.mark.parametrize('kwargs', [dict(speed_mps=0.), dict(confirm_frames=2.5),
                                     dict(curvature_samples=4), dict(max_input_gap_s=float('nan')),
                                     dict(max_heading_deviation_deg=90.)])
def test_invalid_recovery_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        RecoveryConfig(**kwargs)

"""真实节点中恢复行为的仲裁、PID 重建及硬故障保护。"""

from dataclasses import replace
import math
import time

import numpy as np

from process_node.visualization import draw_laser
from test_motion import odom
from test_node_corridor import node, start, tick
from test_node_turning import approach, tick_branch


def break_path(node):
    # 模拟当前位置已无法跟踪旧参考；雷达和反馈仍正常，不伪造可通行候选。
    node.turning._path[:, 0] -= 4.


def test_path_failure_stops_then_recovers_with_independently_checked_curve(node):
    approach(node)
    break_path(node)
    tick_branch(node, 100.6)
    assert node.last_turn.status == 'TURN_FAULT' and node.last_turn.recoverable
    assert node.last_recovery.phase == 'WAIT' and node.speed_value == node.stop_speed
    for t in (100.7, 100.8):
        tick_branch(node, t)
        assert node.speed_value == node.stop_speed
    tick_branch(node, 100.9)
    assert node.last_recovery.phase == 'MOVE'
    assert node.status == 'CONTROL_WARMUP'
    tick_branch(node, 101.)
    assert node.status == 'OK'
    assert node.last_control_input.reference.reference_id == 'local_recovery'
    assert node.last_control_input.reference.target_speed_mps == .06
    assert node.last_recovery.curvature_per_m < 0
    assert node.speed_value < 0 and node.last_plan.target is None
    image = draw_laser(node.last_plan, node.config, node.status, node.steer_value,
                       node.speed_value, node.label, node.last_command, node.last_corridor,
                       node.last_exits, node.last_turn, node.last_recovery)
    assert np.any(np.all(image == [255, 235, 0], axis=2))


def test_a_soft_fault_on_a_new_straight_road_can_handoff_to_cruise(node):
    start(node)
    # 已选出口在车后，整车已经进入新走廊；这正是历史 FAULT 阻断后续直道的情形。
    node.turning.phase = 'TURN'
    node.turning._gate_world = np.array([-.8, 0.])
    node.turning._out_heading = 0.
    node.turning._exit_id = 7
    node.turning.invalidate('old path failed', recoverable=True)
    for t in (100.4, 100.5, 100.6, 100.7):
        tick(node, t)
        assert node.speed_value == node.stop_speed
    assert node.last_recovery.handoff
    assert node.turning.phase == 'CRUISE' and node.turning._passed
    tick(node, 100.8)
    assert node.status == 'CONTROL_WARMUP'
    tick(node, 100.9)
    assert node.status == 'OK'
    assert node.last_control_input.reference.reference_id == 'corridor'


def test_pose_jump_during_recovery_is_a_hard_stop(node):
    approach(node)
    break_path(node)
    tick_branch(node, 100.6)
    node.odometry_callback(odom(100.7, x=10.))
    assert node.speed_value == node.stop_speed
    tick_branch(node, 100.8)
    assert node.recovery.phase == 'FAULT'
    tick_branch(node, 100.9)
    assert not node.last_turn.recoverable
    assert node.last_recovery.heading_rad is None
    assert node.speed_value == node.stop_speed


def test_disabled_recovery_preserves_previous_turn_fault_policy(node):
    node.recovery.config = replace(node.recovery.config, enabled=False)
    approach(node)
    break_path(node)
    for t in (100.6, 100.7, 100.8, 100.9):
        tick_branch(node, t)
        assert node.status == 'TURN_FAULT'
        assert node.speed_value == node.stop_speed


def test_original_time_rewind_cannot_be_recovered(node):
    approach(node)
    tick_branch(node, 100.4)
    assert node.speed_value == node.stop_speed
    tick_branch(node, 100.6)
    assert node.last_turn.status == 'TURN_FAULT' and not node.last_turn.recoverable
    assert node.speed_value == node.stop_speed


def test_watchdog_during_local_motion_stops_and_clears_active_gui_arrow(node):
    approach(node)
    break_path(node)
    for i in range(5):
        tick_branch(node, 100.6+i*.1)
    assert node.status == 'OK'
    node.motion.last_received = time.monotonic()-1.
    node._tick()
    assert node.speed_value == node.stop_speed
    assert node.recovery.phase == 'WAIT'
    assert node.last_recovery is None


def test_active_recovery_does_not_start_or_consume_a_new_side_exit(node, monkeypatch):
    start(node)
    node.recovery._fault_result('recovery budget exhausted')
    def unexpected(*args, **kwargs):
        raise AssertionError('恢复接管时不应启动新的转弯任务')
    monkeypatch.setattr(node.turning, 'update', unexpected)
    for i in range(5):
        tick_branch(node, 100.4+i*.1)
    assert node.turning.phase == 'CRUISE' and not node.turning._passed
    assert node.speed_value == node.stop_speed

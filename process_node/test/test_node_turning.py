"""真实节点中的右转仲裁、控制门控、显示和输入保护。"""

from dataclasses import replace
import math
import time

import numpy as np

from process_node.corridor import CorridorEstimate
from process_node.visualization import draw_laser
from test_exits import road
from test_motion import odom
from test_node_baseline import message
from test_node_corridor import node


def tick_branch(node, t, obstacle=False):
    msg = message()
    msg.urg.angle_increment = 2*math.pi/1440
    ranges = road().ranges.copy()
    if obstacle:
        ranges[0] = .65
    msg.urg.ranges = ranges.astype(np.float32).tolist()
    ns = round(t*1e9)
    msg.urg.header.stamp.sec, msg.urg.header.stamp.nanosec = divmod(ns, 10**9)
    node.odometry_callback(odom(t))
    node.laser_scan_callback(msg)


def approach(node):
    node.odometry_callback(odom(99.9))
    for i in range(6):
        tick_branch(node, 100+i*.1)
    assert node.last_turn.phase == 'APPROACH'
    assert node.last_turn.active
    assert node.status == 'OK'


def test_turn_overrides_cruise_only_after_exit_and_path_validation(node):
    approach(node)
    assert node.last_control_input.reference.reference_id.startswith('right_turn:')
    assert node.last_control_input.reference.target_speed_mps == .1
    assert node.last_plan.target is None  # 曲线没有伪装成直线 Candidate。
    assert node.last_turn.swept_body and node.last_turn.path_body
    assert node.speed_value < 0
    image = draw_laser(node.last_plan, node.config, node.status, node.steer_value,
                       node.speed_value, node.label, node.last_command, node.last_corridor,
                       node.last_exits, node.last_turn)
    assert np.any(np.all(image == [255, 235, 0], axis=2))


def test_committed_path_continues_when_incoming_corridor_is_lost(node, monkeypatch):
    approach(node)
    identifier = node.last_turn.exit_id
    monkeypatch.setattr(node.corridor, 'update', lambda *args: CorridorEstimate('CORRIDOR_LOST', 'no walls'))
    tick_branch(node, 100.6)
    assert node.last_corridor.status == 'CORRIDOR_LOST'
    assert node.last_turn.exit_id == identifier
    assert node.status == 'OK'
    assert not node.last_control_input.reference.stop_requested


def test_curved_path_obstruction_overrides_controller_then_resumes_same_exit(node):
    approach(node)
    identifier = node.last_turn.exit_id
    tick_branch(node, 100.6, obstacle=True)
    assert node.last_turn.status == 'TURN_BLOCKED'
    assert node.speed_value == node.stop_speed
    tick_branch(node, 100.7)
    assert node.last_turn.exit_id == identifier
    assert node.status == 'CONTROL_WARMUP'


def test_watchdog_latches_fault_for_an_active_turn(node):
    approach(node)
    node.last_received = time.monotonic()-node.timeout-1
    node._tick()
    assert node.speed_value == node.stop_speed
    tick_branch(node, 100.6)
    assert node.last_turn.phase == 'FAULT'
    assert node.status == 'TURN_FAULT'
    assert node.speed_value == node.stop_speed


def test_disabled_turning_leaves_original_cruise_reference(node):
    node.turning.config = replace(node.turning.config, enabled=False)
    node.odometry_callback(odom(99.9))
    for i in range(6):
        tick_branch(node, 100+i*.1)
    assert not node.last_turn.active
    assert node.last_control_input.reference.reference_id == 'corridor'

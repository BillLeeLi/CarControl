"""走廊参考接入真实节点回调、PD/PI 和停止保护的集成测试。"""

import math
import time

import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter

from process_node.process_node import LaserWindowAnalyzer
from process_node.visualization import draw_laser
from test_corridor import scene
from test_motion import odom
from test_node_baseline import message


@pytest.fixture
def node():
    context = Context()
    rclpy.init(context=context)
    node = LaserWindowAnalyzer(context=context, namespace='/corridor_test',
                               parameter_overrides=[Parameter('show_visualization', value=False)])
    yield node
    node.destroy_node()
    context.shutdown()


def tick(node, t, y=0.0, yaw=0.0, modify=None):
    msg = message()
    scan = scene(heading=-yaw, center=-y, count=720)
    msg.urg.angle_increment = scan.angle_increment
    msg.urg.ranges = scan.ranges.astype(np.float32).tolist()
    ns = round(t * 1e9)
    msg.urg.header.stamp.sec, msg.urg.header.stamp.nanosec = divmod(ns, 10**9)
    if modify is not None:
        modify(msg)
    node.odometry_callback(odom(t, y=y, yaw=yaw))
    node.laser_scan_callback(msg)
    return msg


def start(node, y=0.0, yaw=0.0):
    node.odometry_callback(odom(99.9, y=y, yaw=yaw))
    for i in range(4):
        tick(node, 100 + .1*i, y=y, yaw=yaw)


def test_default_corridor_confirms_then_warms_up_and_controls(node):
    assert node.reference_mode == 'corridor'
    node.odometry_callback(odom(99.9))
    for t in (100.0, 100.1):
        tick(node, t)
        assert node.status == 'CORRIDOR_ACQUIRING'
        assert node.speed_value == node.stop_speed
    tick(node, 100.2)
    assert node.status == 'CONTROL_WARMUP'
    tick(node, 100.3)
    assert node.status == 'OK'
    assert node.last_corridor.status == 'CORRIDOR_TRACKING'
    assert node.last_plan.target.angle == pytest.approx(0, abs=.01)
    assert node.speed_value == -6
    assert node.last_control_input.reference.reference_id == 'corridor'


def test_skipped_scans_do_not_restart_cruise_or_controller(node):
    start(node)
    for t in (100.64, 100.98, 101.32, 101.66):
        tick(node, t)
        assert node.last_corridor.status == 'CORRIDOR_TRACKING'
        assert node.status == 'OK'


@pytest.mark.parametrize('y,yaw', [(.2, 0.0), (0.0, math.radians(10))])
def test_lateral_or_heading_error_generates_right_correction(node, y, yaw):
    start(node, y, yaw)
    assert node.status == 'OK'
    assert node.last_plan.target.angle < 0
    assert node.steer_value > node.steer_center  # steer_sign=-1，右纠偏对应正控制偏移。


def test_single_wall_and_missing_walls_reduce_speed_then_expire(node):
    start(node)
    def single(msg):
        angles = np.arange(len(msg.urg.ranges))*msg.urg.angle_increment
        msg.urg.ranges = np.where(np.sin(angles)>0, np.inf, msg.urg.ranges).astype(np.float32).tolist()
    tick(node, 100.4, modify=single)
    assert node.last_corridor.status == 'CORRIDOR_SINGLE_WALL'
    assert node.last_control_input.reference.target_speed_mps == .1
    def missing(msg):
        msg.urg.ranges = [10.0]*720  # 在墙面拟合范围外有有限回波，仍能测试历史超时。
    tick(node, 100.5, modify=missing)
    assert node.last_corridor.status == 'CORRIDOR_HOLD'
    assert node.last_control_input.reference.target_speed_mps == .1
    for t in (100.6, 100.7, 100.8):
        tick(node, t, modify=missing)
    assert node.last_corridor.status == 'CORRIDOR_LOST'
    assert node.status.startswith('RECOVERY_')
    assert node.speed_value == node.stop_speed
    tick(node, 100.9)
    assert node.last_corridor.status == 'CORRIDOR_ACQUIRING'
    assert node.last_recovery.heading_rad is None


def test_side_candidates_do_not_bypass_a_blocked_cruise_reference(node):
    start(node)
    def obstacle(msg):
        msg.urg.ranges[0] = 1.3
    tick(node, 100.4, modify=obstacle)
    assert node.last_plan.candidates
    assert node.last_recovery.cause == 'CRUISE_BLOCKED'
    assert node.status == 'RECOVERY_WAITING'
    assert node.last_plan.target is None
    assert node.speed_value == node.stop_speed
    tick(node, 100.5)
    assert node.status == 'CONTROL_WARMUP'


@pytest.mark.parametrize('loss', ['sensor', 'odom'])
def test_watchdog_clears_corridor_and_requires_reconfirmation(node, loss):
    start(node)
    if loss == 'sensor':
        node.last_received = time.monotonic()-node.timeout-1
    else:
        node.motion.last_received = time.monotonic()-node.motion.config.timeout_s-1
    node._tick()
    assert node.speed_value == node.stop_speed
    assert node.last_corridor is None
    tick(node, 100.4)
    assert node.status == 'CORRIDOR_ACQUIRING'


def test_duplicate_scan_stops_and_does_not_reuse_reference(node):
    msg = None
    node.odometry_callback(odom(99.9))
    for t in (100, 100.1, 100.2, 100.3):
        msg = tick(node, t)
    node.laser_scan_callback(msg)
    assert node.status == 'CORRIDOR_TIME_INVALID'
    assert node.speed_value == node.stop_speed
    tick(node, 100.4)
    assert node.status == 'CORRIDOR_ACQUIRING'


def test_invalid_metadata_cannot_leave_an_active_corridor_reference(node):
    start(node)
    def invalid(msg):
        msg.urg.angle_increment = float('nan')
    tick(node, 100.4, modify=invalid)
    assert node.status == 'INVALID_METADATA'
    assert node.last_corridor is None
    assert node.speed_value == node.stop_speed


def test_diagnostic_image_includes_corridor_model_without_a_gui(node):
    start(node, y=.2)
    image = draw_laser(node.last_plan, node.config, node.status, node.steer_value,
                       node.speed_value, node.label, node.last_command, node.last_corridor)
    assert image.shape == (900, 800, 3)
    assert np.any(np.all(image == [210, 110, 230], axis=2))  # 紫色中线/参考可被实际绘制。

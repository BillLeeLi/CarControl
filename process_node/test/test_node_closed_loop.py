"""闭环模式接线、直路选择以及独立看门狗复位的集成检查。"""

import math
import time

import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter

from process_node.pid_controller import PIDController
from process_node.process_node import LaserWindowAnalyzer
from test_motion import odom
from test_node_baseline import message


@pytest.fixture
def node():
    context = Context()
    rclpy.init(context=context)
    node = LaserWindowAnalyzer(
        context=context, namespace='/closed_loop_test',
        parameter_overrides=[Parameter('show_visualization', value=False)],
    )
    yield node
    node.destroy_node()
    context.shutdown()


def scan_at(t):
    msg = message()
    ns = round(t * 1e9)
    msg.urg.header.stamp.sec = ns // 10**9
    msg.urg.header.stamp.nanosec = ns % 10**9
    return msg


def start(node):
    node.odometry_callback(odom(99.9))
    node.odometry_callback(odom(100))
    node.laser_scan_callback(scan_at(100))
    assert node.status == 'CONTROL_WARMUP'
    node.odometry_callback(odom(100.1))
    node.laser_scan_callback(scan_at(100.1))
    assert node.status == 'OK'


def test_default_mode_runs_pi_with_physical_target_and_publishes_limited_output(node):
    assert isinstance(node.controller, PIDController)
    start(node)
    assert node.last_control_input.reference.target_speed_mps == 0.2
    assert node.last_command.diagnostics.measured_speed_mps == 0
    assert node.steer_value == node.steer_center
    assert node.speed_value == -6


@pytest.mark.parametrize('loss', ['laser', 'odom'])
def test_watchdogs_stop_and_recovery_warms_up_from_zero(node, loss):
    start(node)
    assert node.speed_value != 0
    if loss == 'laser':
        node.last_received = time.monotonic() - node.timeout - 1
    else:
        node.motion.last_received = time.monotonic() - node.motion.config.timeout_s - 1
    node._tick()
    assert node.speed_value == node.stop_speed
    assert node.last_command.diagnostics is None
    node.odometry_callback(odom(100.2))
    node.laser_scan_callback(scan_at(100.2))
    assert node.status == 'CONTROL_WARMUP'
    node.odometry_callback(odom(100.3))
    node.laser_scan_callback(scan_at(100.3))
    assert node.last_command.diagnostics.target_speed_mps == pytest.approx(0.02)


def test_straight_mode_rejects_a_side_only_opening(node):
    node.reference_mode = 'straight'
    node.odometry_callback(odom(99.9))
    node.odometry_callback(odom(100))
    msg = scan_at(100)
    angles = (np.arange(720) * msg.urg.angle_increment + math.pi) % (2 * math.pi) - math.pi
    msg.urg.ranges = np.where(np.abs(angles - math.radians(30)) < math.radians(36),
                              8.0, 1.0).astype(np.float32).tolist()
    node.laser_scan_callback(msg)
    assert node.last_plan.candidates  # 存在侧向候选，但不能拿它冒充直行路径
    assert node.status == 'NO_STRAIGHT_PATH'
    assert node.last_plan.target is None
    assert node.speed_value == node.stop_speed


def test_straight_mode_selects_zero_and_zero_target_stops(node):
    node.reference_mode = 'straight'
    start(node)
    assert node.last_plan.target.angle == 0
    node.target_speed_mps = 0.0
    node.odometry_callback(odom(100.2))
    node.laser_scan_callback(scan_at(100.2))
    assert node.status == 'STOP_REQUESTED'
    assert node.speed_value == node.stop_speed


def test_duplicate_scan_does_not_integrate_again(node):
    start(node)
    node.laser_scan_callback(scan_at(100.1))
    assert node.status == 'CONTROL_TIME_INVALID'
    assert node.speed_value == node.stop_speed

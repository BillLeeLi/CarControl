"""检查 ROS 集成的控制限幅、停止和回放恢复，测试话题与实车命名空间隔离。"""

import math
import time

import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from robotsdk_msgs.msg import AllSensor

from process_node.process_node import LaserWindowAnalyzer
from process_node.window_monitor import WindowMonitor


@pytest.fixture
def node():
    context = Context()
    rclpy.init(context=context)
    node = LaserWindowAnalyzer(
        context=context, namespace='/baseline_test',
        parameter_overrides=[Parameter('show_visualization', value=False)])
    yield node
    node.destroy_node()
    context.shutdown()


def message():
    msg = AllSensor()
    msg.urg.angle_min = 0.0
    msg.urg.angle_increment = 0.008726646
    msg.urg.range_min = 0.05
    msg.urg.range_max = 50.0
    msg.urg.ranges = [10.0] * 720
    msg.urg.header.stamp.sec = 100
    return msg


def test_core_maps_a_real_angle_and_stops_without_valid_data(node):
    scan = message().urg
    steer, speed = node.process_core(scan)
    assert steer == node.steer_center
    assert speed == node.cruise_speed
    scan.ranges = [float('inf')] * 720
    assert node.process_core(scan) == (node.steer_center, node.stop_speed)
    assert node.previous_target is None


def test_watchdog_stops_even_without_another_sensor_callback(node, monkeypatch):
    sent = []
    monkeypatch.setattr(node, '_publish', lambda steer, speed: sent.append((steer, speed)))
    node.last_received = time.monotonic() - node.timeout - 1
    node.previous_target = 0.5
    node._tick()
    assert sent[-1] == (node.steer_center, node.stop_speed)
    assert node.status == 'STALE_DATA'
    assert node.previous_target is None


def test_timestamp_rewind_discards_old_target(node, monkeypatch):
    monkeypatch.setattr(node, '_publish', lambda steer, speed: None)
    node.last_stamp_ns = 200 * 10**9
    node.previous_target = 0.9
    node.laser_scan_callback(message())
    assert node.previous_target == pytest.approx(0)


def test_steering_limit_applies_to_offset_from_calibrated_center(node):
    node.steer_center = 20
    node.steer_gain = -100000.0
    msg = message()
    # 固定一个左侧可见开口，验证转向符号及中位附近限幅。
    angles = (np.arange(720) * msg.urg.angle_increment + math.pi) % (2 * math.pi) - math.pi
    msg.urg.ranges = np.where(np.abs(angles - math.radians(30)) < math.radians(36),
                              8.0, 1.0).astype(np.float32).tolist()
    steer, _ = node.process_core(msg.urg)
    assert node.last_plan.target is not None
    assert steer == node.steer_center - node.steer_limit


@pytest.mark.parametrize('backend_error', [False, True])
def test_closed_window_requests_stop_and_exit_without_recreating_it(
        node, monkeypatch, backend_error):
    from process_node import process_node as module
    sent = []
    node.gui = True
    node.windows_created = True
    node.window_monitor = WindowMonitor((node.CAMERA_WINDOW, node.LASER_WINDOW))
    node.window_monitor.seen.update((node.CAMERA_WINDOW, node.LASER_WINDOW))
    node.last_received = time.monotonic()
    monkeypatch.setattr(node, '_publish', lambda steer, speed: sent.append((steer, speed)))
    def visibility(*args):
        if backend_error:
            raise module.cv2.error('window destroyed')
        return 0.0
    monkeypatch.setattr(module.cv2, 'getWindowProperty', visibility)
    monkeypatch.setattr(module.cv2, 'imshow', lambda *args: pytest.fail('不能重建已关闭窗口'))
    node._tick()
    assert node.exit_requested
    assert sent[-1] == (node.steer_center, node.stop_speed)

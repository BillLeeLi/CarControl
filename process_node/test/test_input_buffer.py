"""验证 GUI/计算延迟时输入有界、保鲜时间和批量关联，而不依赖真实桌面。"""

import threading
import time

import pytest
from rclpy.context import Context
from rclpy.parameter import Parameter
import rclpy

from process_node.input_buffer import InputBuffer
from process_node.process_node import LaserWindowAnalyzer
from test_motion import odom
from test_node_baseline import message


def test_receiver_can_collect_odometry_while_main_thread_is_busy():
    inputs = InputBuffer()
    busy = threading.Event()
    def receive():
        busy.wait()
        for i in range(15):
            inputs.receive_odometry(i, received_at=10 + i*.02)
            inputs.receive_sensor(i, received_at=10 + i*.02)
    worker = threading.Thread(target=receive)
    worker.start()
    busy.set()
    worker.join(timeout=2)
    assert not worker.is_alive()
    odometry, sensor, overflow = inputs.take()
    assert [x.message for x in odometry] == list(range(15))
    assert sensor.message == 14
    assert sensor.received_at == pytest.approx(10.28)
    assert inputs.skipped_scans == 14
    assert not overflow


def test_overflow_is_bounded_and_reported_for_motion_reset():
    inputs = InputBuffer(odom_capacity=3)
    for i in range(5):
        inputs.receive_odometry(i)
    odometry, sensor, overflow = inputs.take()
    assert [x.message for x in odometry] == [2,3,4]
    assert overflow and sensor is None
    assert inputs.take() == ([], None, False)


@pytest.fixture
def node():
    context = Context()
    rclpy.init(context=context)
    n = LaserWindowAnalyzer(context=context, namespace='/buffer_test',
        parameter_overrides=[Parameter('show_visualization', value=False),
                             Parameter('reference_mode', value='perception')])
    yield n
    n.destroy_node()
    context.shutdown()


def test_queued_scan_uses_the_full_odometry_batch_before_computing(node):
    now = time.monotonic()
    node.inputs.receive_odometry(odom(99.9), received_at=now)
    # 激光先进入缓冲，但配套里程计在主线程开始计算前已到达。
    node.inputs.receive_sensor(message(), received_at=now)
    for t in (100,100.02,100.04):
        node.inputs.receive_odometry(odom(t), received_at=now)
    node.process_pending_inputs()
    assert node.last_feedback.valid
    assert node.status == 'CONTROL_WARMUP'
    assert node.last_feedback.skew_s == pytest.approx(0)


def test_processing_delay_does_not_refresh_odometry_receipt_time(node):
    before = time.monotonic() - node.motion.config.timeout_s - .1
    node.inputs.receive_odometry(odom(99.9), received_at=before)
    node.inputs.receive_odometry(odom(100), received_at=before)
    node.inputs.receive_sensor(message())
    node.process_pending_inputs()
    assert node.motion.last_received == before
    assert node.last_feedback.status == 'STALE_ODOM'
    assert node.speed_value == node.stop_speed


def test_obsolete_queued_scan_is_not_processed_or_published_as_motion(node):
    node.inputs.receive_sensor(message(), received_at=time.monotonic()-node.timeout-1)
    node.process_pending_inputs()
    assert node.last_plan is None
    node._tick()
    assert node.status == 'STALE_DATA'
    assert node.speed_value == node.stop_speed


def test_publishing_rechecks_receipt_freshness_after_a_slow_compute(node, monkeypatch):
    now = time.monotonic()
    node.odometry_callback(odom(99.9), received_at=now)
    node.odometry_callback(odom(100), received_at=now)
    original = node.process_core
    def slow_core(scan):
        result = original(scan)
        node.last_received = time.monotonic()-node.timeout-1
        return result
    monkeypatch.setattr(node, 'process_core', slow_core)
    node.laser_scan_callback(message())
    assert node.status == 'STALE_DATA'
    assert node.speed_value == node.stop_speed


def test_repeated_idle_ticks_only_transmit_images_once_but_keep_window_events(node, monkeypatch):
    import process_node.process_node as module
    from process_node.window_monitor import WindowMonitor
    node.gui = True
    node.window_monitor = WindowMonitor((node.CAMERA_WINDOW, node.LASER_WINDOW))
    sends=[];events=[]
    monkeypatch.setattr(module.cv2, 'namedWindow', lambda *a: None)
    monkeypatch.setattr(module.cv2, 'imshow', lambda *a: sends.append(a[0]))
    monkeypatch.setattr(module.cv2, 'waitKey', lambda *a: events.append(1) or -1)
    monkeypatch.setattr(module.cv2, 'getWindowProperty', lambda *a: -1)
    for _ in range(5):node._tick()
    assert len(sends) == 2
    assert len(events) == 5

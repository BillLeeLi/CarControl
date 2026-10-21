"""离线激光感知基线；当前输出是方向映射，尚未实现行为式绕障和双闭环。"""

from dataclasses import fields
import math
import os
import signal
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from robotsdk_msgs.msg import AllSensor, Control

from process_node.perception import PlannerConfig, analyze_scan
from process_node.visualization import draw_laser, draw_steering
from process_node.window_monitor import create_window_monitor


class LaserWindowAnalyzer(Node):
    """保留原模板类名和 process_core 接口，便于和课程程序集成。"""

    CAMERA_WINDOW = "Steering Visualization"
    LASER_WINDOW = "Laser Diagnostics"

    def __init__(self, **kwargs):
        super().__init__("laser_window_analyzer", **kwargs)
        defaults = PlannerConfig()
        for item in fields(defaults):
            self.declare_parameter(item.name, getattr(defaults, item.name))
        self.config = PlannerConfig(
            **{
                item.name: self.get_parameter(item.name).value
                for item in fields(defaults)
            }
        )
        self.declare_parameter("sensorTopic", "/sync_store_node/allsensor")
        self.declare_parameter("group_label", "midterm-baseline")
        self.declare_parameter("show_visualization", True)
        self.declare_parameter("steer_center", 0)
        self.declare_parameter("steer_limit", 600)
        self.declare_parameter("steer_gain_per_rad", -300.0)
        self.declare_parameter("cruise_speed", -100)
        self.declare_parameter("stop_speed", 0)
        self.declare_parameter("sensor_timeout_s", 1.0)
        self.declare_parameter("log_period_s", 1.0)
        self.steer_center = self.get_parameter("steer_center").value
        self.steer_limit = self.get_parameter("steer_limit").value
        self.steer_gain = self.get_parameter("steer_gain_per_rad").value
        self.cruise_speed = self.get_parameter("cruise_speed").value
        self.stop_speed = self.get_parameter("stop_speed").value
        self.timeout = self.get_parameter("sensor_timeout_s").value
        self.log_period = self.get_parameter("log_period_s").value
        if (
            not 0 < self.steer_limit <= 32767
            or abs(self.steer_center) + self.steer_limit > 32767
            or any(
                not -32768 <= x <= 32767 for x in (self.cruise_speed, self.stop_speed)
            )
            or not math.isfinite(self.steer_gain)
            or not math.isfinite(self.timeout)
            or self.timeout <= 0
            or not math.isfinite(self.log_period)
            or self.log_period <= 0
        ):
            raise ValueError("控制量必须符合 int16 范围，超时及日志周期必须为正有限值")

        self.label = self.get_parameter("group_label").value
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Control, "control", 10)
        self.subscription = self.create_subscription(
            AllSensor,
            self.get_parameter("sensorTopic").value,
            self.laser_scan_callback,
            10,
        )
        self.last_plan = None
        self.previous_target = None
        self.last_received = None
        self.last_stamp_ns = None
        self.last_log_time = -math.inf
        self.last_log_status = None
        self.last_stop_time = -math.inf
        self.steer_value = self.steer_center
        self.speed_value = self.stop_speed
        self.status = "WAITING"
        self.exit_requested = False
        self.camera_image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.laser_image = None
        self.gui = self.get_parameter("show_visualization").value
        self.windows_created = False
        self.window_monitor = None
        if self.gui and not (
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        ):
            self.get_logger().warning("没有图形桌面，自动关闭窗口；可继续观察终端输出")
            self.gui = False

        # 独立的单调时钟定时器：bag 播完或暂停后，仍处理窗口事件及输入超时。
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.ui_timer = self.create_timer(0.05, self._tick, clock=self.steady_clock)
        self.get_logger().info(
            f"[{self.label}] 激光感知基线；control={self.publisher_.topic_name}。"
            "尺寸、停止值和转向映射为离线默认值，上车前需标定。"
        )

    def _publish(self, steer, speed):
        control = Control()
        control.header.stamp = self.get_clock().now().to_msg()
        control.header.frame_id = "base_link"
        control.steer = int(steer)
        control.speed = int(speed)
        self.publisher_.publish(control)
        self.steer_value, self.speed_value = int(steer), int(speed)

    def publish_stop(self):
        """停止值由参数给出；必须通过实际小车标定确认其含义。"""
        self._publish(self.steer_center, self.stop_speed)

    def laser_scan_callback(self, msg):
        stamp = msg.urg.header.stamp
        stamp_ns = stamp.sec * 10**9 + stamp.nanosec
        if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
            self.previous_target = None  # 循环/重新回放时，不延续上次末帧目标。
        self.last_stamp_ns = stamp_ns
        self.last_received = time.monotonic()
        steer, speed = self.process_core(msg.urg)
        self._publish(steer, speed)
        self._log_decision()
        if self.gui:
            try:
                # 保留课程模板中的水平翻转；判断物理左右应以激光图坐标为准。
                image = self.bridge.imgmsg_to_cv2(
                    msg.xtion_color, desired_encoding="bgr8"
                )
                self.camera_image = cv2.flip(image, 1)
            except (CvBridgeError, ValueError, cv2.error) as error:
                self.get_logger().warning(
                    f"RGB 显示失败，激光处理仍继续：{error}", throttle_duration_sec=5.0
                )
            self.laser_image = draw_laser(
                self.last_plan,
                self.config,
                self.status,
                self.steer_value,
                self.speed_value,
                self.label,
            )

    def _log_decision(self):
        now = time.monotonic()
        if (
            self.status == self.last_log_status
            and now - self.last_log_time < self.log_period
        ):
            return
        self.last_log_time, self.last_log_status = now, self.status
        result = self.last_plan
        if result is None or self.status == "STALE_DATA":
            self.get_logger().info(
                f"[{self.label}] {self.status}：等待有效传感器输入，输出停止值"
            )
            return
        data = result.scan
        target = result.target
        direction = "--" if target is None else f"{math.degrees(target.angle):+.2f}deg"
        clearance = "--" if target is None else f"{target.clearance_m:.2f}m"
        support = "--" if target is None else f"{target.valid_ratio:.0%}"
        mean = "--" if target is None else f"{target.mean_distance_m:.2f}m"
        front = (
            "unknown"
            if result.front_clearance_m is None
            else f"{result.front_clearance_m:.2f}m"
        )
        self.get_logger().info(
            f"[{self.label}] {self.status} | valid={data.valid_ratio:.1%}"
            f" ({int(data.valid.sum())}/{data.ranges.size}) +inf={int(data.positive_inf.sum())}"
            f" invalid={int((~data.valid & ~data.positive_inf).sum())}"
            f" | target={direction} support={support} finite_mean={mean}"
            f" clearance={clearance} front={front}"
            f" gaps={len(result.gaps)} candidates={len(result.candidates)}"
            f" | steer={self.steer_value} speed={self.speed_value} | {result.reason}"
        )

    def _tick(self):
        now = time.monotonic()
        if self.last_received is None or now - self.last_received > self.timeout:
            self.status = "WAITING" if self.last_received is None else "STALE_DATA"
            self.previous_target = None
            if now - self.last_stop_time >= 0.2:
                self.publish_stop()
                self.last_stop_time = now
            self._log_decision()
            self.laser_image = None
        if not self.gui:
            return
        try:
            if self.window_monitor is None:
                self.window_monitor = create_window_monitor(
                    (self.CAMERA_WINDOW, self.LASER_WINDOW)
                )
            if self.windows_created:
                # 必须先检查，再 imshow，否则点叉后 imshow 会重新创建窗口。
                if self.window_monitor.any_closed():
                    self.get_logger().info("检测到窗口关闭，发送停止值并退出")
                    self.publish_stop()
                    self.exit_requested = True
                    return
            else:
                for name in (self.CAMERA_WINDOW, self.LASER_WINDOW):
                    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
                self.windows_created = True
            image = self.camera_image.copy()
            target = self.previous_target if self.status == "OK" else None
            draw_steering(
                image,
                self.steer_value,
                self.steer_center,
                self.steer_limit,
                self.label,
                self.status,
                target,
            )
            cv2.imshow(self.CAMERA_WINDOW, image)
            laser = self.laser_image
            if laser is None:
                laser = draw_laser(
                    self.last_plan,
                    self.config,
                    self.status,
                    self.steer_value,
                    self.speed_value,
                    self.label,
                )
            cv2.imshow(self.LASER_WINDOW, laser)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                self.publish_stop()
                self.exit_requested = True
            # waitKey 处理完 GTK 创建事件后记录已打开的窗口；不能仅靠
            # namedWindow 调用成功，就假定可见性查询已经得到有效结果。
            elif self.window_monitor.any_closed():
                self.get_logger().info("检测到窗口关闭，发送停止值并退出")
                self.publish_stop()
                self.exit_requested = True
        except cv2.error as error:
            self.get_logger().error(f"窗口显示失败，切换终端模式：{error}")
            self.gui = False
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass

    def process_core(self, urg):
        """输入 LaserScan，返回 (steer, speed)，保持课程模板接口。"""
        self.last_plan = analyze_scan(urg, self.config, self.previous_target)
        self.status = self.last_plan.status
        if self.last_plan.target is None:
            self.previous_target = None
            return self.steer_center, self.stop_speed
        target = self.last_plan.target.angle
        self.previous_target = target
        # 这里只把弧度映射成控制量，增益单位为“控制单位/弧度”。
        # 尚未加入 PD、速度 PI 或车辆转弯运动学；当前目标是检查选方向依据。
        delta = int(
            round(
                np.clip(self.steer_gain * target, -self.steer_limit, self.steer_limit)
            )
        )
        return self.steer_center + delta, self.cruise_speed

def main(args=None):
    # 保持 ROS 上下文有效，先发送停止值再退出。launch 和终端可能重复发 SIGINT，
    # 信号处理器只设置标志，避免第二次 Ctrl+C 打断 destroy_node 的清理过程。
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    previous_handlers = {
        sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)
    }
    node = None
    try:
        node = LaserWindowAnalyzer()
        while rclpy.ok() and not node.exit_requested and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_stop()
            node.destroy_node()
        if node is not None and node.windows_created:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        rclpy.try_shutdown()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    main()

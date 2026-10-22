"""激光目标方向跟踪、独立运动反馈、方向 PD 与低速前进 PI。"""

from dataclasses import fields, replace
import math
import os
import signal
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from robotsdk_msgs.msg import AllSensor, Control

from process_node.control import (
    BaselineControlConfig, BaselineController, ControlInput, ControlReference,
)
from process_node.motion import MotionConfig, MotionEstimator
from process_node.pid_controller import PIDConfig, PIDController
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
        self.declare_parameter("group_label", "midterm-inf-v2")
        self.declare_parameter("show_visualization", True)
        self.declare_parameter("control_mode", "closed_loop")
        self.declare_parameter("reference_mode", "perception")
        self.declare_parameter("target_speed_mps", 0.2)
        self.control_mode = self.get_parameter("control_mode").value
        self.reference_mode = self.get_parameter("reference_mode").value
        self.target_speed_mps = self.get_parameter("target_speed_mps").value
        control_defaults = BaselineControlConfig()
        for item in fields(control_defaults):
            self.declare_parameter(item.name, getattr(control_defaults, item.name))
        baseline_config = BaselineControlConfig(**{
            item.name: self.get_parameter(item.name).value for item in fields(control_defaults)
        })
        pid_defaults = PIDConfig()
        for item in fields(pid_defaults):
            if not self.has_parameter(item.name):
                self.declare_parameter(item.name, getattr(pid_defaults, item.name))
        pid_config = PIDConfig(**{
            item.name: self.get_parameter(item.name).value for item in fields(pid_defaults)
        })
        if self.control_mode not in ('closed_loop', 'baseline'):
            raise ValueError('control_mode 必须为 closed_loop 或 baseline')
        if self.reference_mode not in ('perception', 'straight'):
            raise ValueError('reference_mode 必须为 perception 或 straight')
        if (not math.isfinite(self.target_speed_mps)
                or not 0 <= self.target_speed_mps <= pid_config.max_target_speed_mps):
            raise ValueError('target_speed_mps 必须在 [0, max_target_speed_mps] 内')
        if self.control_mode == 'closed_loop' and not baseline_config.require_motion_feedback:
            raise ValueError('closed_loop 必须开启 require_motion_feedback；离线感知观察请选 baseline')
        self.controller = PIDController(pid_config) if self.control_mode == 'closed_loop' \
            else BaselineController(baseline_config)
        self.declare_parameter("odom_topic", "/encoder_imu_node/odom")
        motion_defaults = MotionConfig()
        for item in fields(motion_defaults):
            self.declare_parameter("odom_" + item.name, getattr(motion_defaults, item.name))
        self.motion = MotionEstimator(MotionConfig(**{
            item.name: self.get_parameter("odom_" + item.name).value
            for item in fields(motion_defaults)
        }))
        self.declare_parameter("sensor_timeout_s", 1.0)
        self.declare_parameter("log_period_s", 1.0)
        self.steer_center = self.get_parameter("steer_center").value
        self.steer_limit = self.get_parameter("steer_limit").value
        self.cruise_speed = self.get_parameter("cruise_speed").value
        self.stop_speed = self.get_parameter("stop_speed").value
        self.require_motion_feedback = baseline_config.require_motion_feedback
        self.timeout = self.get_parameter("sensor_timeout_s").value
        self.log_period = self.get_parameter("log_period_s").value
        if (
            not math.isfinite(self.timeout)
            or self.timeout <= 0
            or not math.isfinite(self.log_period)
            or self.log_period <= 0
        ):
            raise ValueError("超时及日志周期必须为正有限值")

        self.label = self.get_parameter("group_label").value
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Control, "control", 10)
        self.subscription = self.create_subscription(
            AllSensor,
            self.get_parameter("sensorTopic").value,
            self.laser_scan_callback,
            10,
        )
        self.odometry_subscription = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odometry_callback, 50,
        )
        self.last_plan = None
        self.previous_target = None
        self.last_received = None
        self.last_stamp_ns = None
        self.last_feedback = self.motion.feedback()
        self.last_control_input = None
        self.last_command = self.controller.stop("WAITING", "等待输入")
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
            self.get_logger().warning(
                "show_visualization=true，但当前进程未设置 DISPLAY/WAYLAND_DISPLAY，"
                "自动关闭窗口。若容器已有桌面，请从桌面终端启动，或设置该桌面的 "
                "DISPLAY 后重启节点；仅设置 show_visualization=true 不会补齐显示环境。"
            )
            self.gui = False

        # 独立的单调时钟定时器：bag 播完或暂停后，仍处理窗口事件及输入超时。
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.ui_timer = self.create_timer(0.05, self._tick, clock=self.steady_clock)
        self.get_logger().info(
            f"[{self.label}] control_mode={self.control_mode}，reference_mode={self.reference_mode}；"
            f"control={self.publisher_.topic_name}，"
            f"odom={self.odometry_subscription.topic_name}，"
            f"require_motion_feedback={self.require_motion_feedback}。"
            "尺寸、控制符号、停止值、前馈和控制增益上车前需标定。"
        )

    def _publish(self, steer, speed):
        control = Control()
        control.header.stamp = self.get_clock().now().to_msg()
        control.header.frame_id = self.motion.config.child_frame_id
        control.steer = int(steer)
        control.speed = int(speed)
        self.publisher_.publish(control)
        self.steer_value, self.speed_value = int(steer), int(speed)

    def publish_stop(self):
        """停止值由参数给出；必须通过实际小车标定确认其含义。"""
        self.last_command = self.controller.stop(self.status, "停止请求覆盖普通控制输出")
        self._publish(self.last_command.steer, self.last_command.speed)

    def odometry_callback(self, msg):
        """运动反馈独立更新，不使用 AllSensor 中的里程计或 IMU 字段。"""
        now = time.monotonic()
        generation = self.motion.generation
        self.motion.update(msg, received_at=now)
        if self.motion.generation != generation:
            self.previous_target = None
            self.controller.reset()
        sensor_fresh = self.last_received is not None and now - self.last_received <= self.timeout
        # 与激光回调/定时器使用同一种关联结果，避免日志在“最新里程计有效”
        # 和“关联到本帧激光的反馈无效”之间反复切换。
        self.last_feedback = self.motion.feedback(
            self.last_stamp_ns if sensor_fresh else None, now=now,
        )
        if (self.require_motion_feedback and not self.last_feedback.valid
                and sensor_fresh):
            self.status = self.last_feedback.status
            self.previous_target = None
            self.publish_stop()
            self.last_stop_time = now
            self.laser_image = None
        self._log_decision()

    def laser_scan_callback(self, msg):
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
                command=self.last_command,
            )

    def _log_decision(self):
        now = time.monotonic()
        log_status = (self.status, self.last_feedback.status)
        if (
            log_status == self.last_log_status
            and now - self.last_log_time < self.log_period
        ):
            return
        self.last_log_time, self.last_log_status = now, log_status
        feedback = self.last_feedback
        motion_text = f"odom={feedback.status}"
        if feedback.state is not None:
            state = feedback.state
            motion_text += (
                f" v={state.speed_mps:+.3f}m/s raw_v={state.raw_speed_mps:+.3f}m/s"
                f" yaw={math.degrees(state.yaw_rad):+.1f}deg"
                f" turn={math.degrees(state.heading_change_rad):+.1f}deg"
                f" travel={state.distance_m:.3f}m signed={state.signed_distance_m:+.3f}m"
                f" local=({state.local_x_m:+.3f},{state.local_y_m:+.3f})m"
                f" dt={state.dt_s:.3f}s"
            )
        if feedback.skew_s is not None:
            motion_text += f" skew={feedback.skew_s:+.3f}s"
        if not feedback.valid:
            motion_text += f" ({feedback.reason}；数值如有显示，仅供诊断)"
        control_text = f"controller={self.control_mode}: {self.last_command.reason}"
        diagnostics = self.last_command.diagnostics
        if diagnostics is not None:
            control_text += (
                f" | v_ref={diagnostics.target_speed_mps:.3f}m/s"
                f" requested={diagnostics.requested_speed_mps:.3f}m/s"
                f" v_used={diagnostics.measured_speed_mps:+.3f}m/s"
                f" e_v={diagnostics.speed_error_mps:+.3f}m/s"
                f" speed[FF,P,I]=({diagnostics.speed_feedforward:.2f},"
                f"{diagnostics.speed_p:.2f},{diagnostics.speed_i:.2f})"
                f" steer[P,D]=({diagnostics.steer_p:.2f},{diagnostics.steer_d:.2f})"
                f" control_dt={diagnostics.dt_s:.3f}s"
                f" sat[steer,speed]=({diagnostics.steer_saturated},{diagnostics.speed_saturated})"
                f" rate[steer,speed]=({diagnostics.steer_rate_limited},"
                f"{diagnostics.speed_rate_limited}) I_frozen={diagnostics.integral_frozen}"
                f" D_reset={diagnostics.derivative_reset}"
            )
        result = self.last_plan
        if result is None or self.status == "STALE_DATA":
            self.get_logger().info(
                f"[{self.label}] {self.status}：等待有效传感器输入，输出停止值 | {motion_text}"
                f" | {control_text}"
            )
            return
        data = result.scan
        target = result.target
        direction = "--" if target is None else f"{math.degrees(target.angle):+.2f}deg"
        clearance = "--" if target is None else f"{target.clearance_m:.2f}m"
        support = "--" if target is None else f"{target.support_ratio:.0%}"
        finite = "--" if target is None else f"{target.valid_ratio:.0%}"
        no_return = "--" if target is None else f"{target.no_return_ratio:.0%}"
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
            f" | target={direction} support={support} finite={finite}"
            f" no_return={no_return} planning_mean={mean}"
            f" no_return_limit={data.no_return_limit_m:.2f}m"
            f" clearance={clearance} front={front}"
            f" gaps={len(result.gaps)} candidates={len(result.candidates)}"
            f" | steer={self.steer_value} speed={self.speed_value} | {motion_text}"
            f" | perception={result.status}: {result.reason}"
            f" | {control_text}"
        )

    def _tick(self):
        now = time.monotonic()
        if self.last_received is None or now - self.last_received > self.timeout:
            self.status = "WAITING" if self.last_received is None else "STALE_DATA"
            self.previous_target = None
            if now - self.last_stop_time >= 0.2:
                self.publish_stop()
                self.last_stop_time = now
            self.laser_image = None
            self.last_feedback = self.motion.feedback(now=now)
        else:
            # 即使激光还在发送、/clock 没有前进，也不能继续使用停止更新的反馈。
            self.last_feedback = self.motion.feedback(self.last_stamp_ns, now=now)
            if self.require_motion_feedback and not self.last_feedback.valid:
                self.status = self.last_feedback.status
                self.previous_target = None
                if now - self.last_stop_time >= 0.2:
                    self.publish_stop()
                    self.last_stop_time = now
                self.laser_image = None
        self._log_decision()
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
                    command=self.last_command,
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
        stamp = urg.header.stamp
        stamp_ns = stamp.sec * 10**9 + stamp.nanosec
        if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
            self.previous_target = None
            self.controller.reset()
            # 新一轮 odom 可能先于激光到达，只有仍留着旧时间段的反馈时才清空。
            if (self.motion.state is not None and self.motion.state.stamp_ns
                    > stamp_ns + round(self.motion.config.scan_tolerance_s * 1e9)):
                self.motion.reset('ODOM_TIME_REWIND', '激光时间回跳，等待同一时间段的新反馈')
        self.last_stamp_ns = stamp_ns
        self.last_plan = analyze_scan(urg, self.config, self.previous_target)
        if self.reference_mode == 'straight' and self.last_plan.target is not None:
            # 直路测试只接受通过了原有观测及车体检查的 0° 候选，不强行向前。
            straight = next((item for item in self.last_plan.candidates
                             if abs(item.angle) < 1e-9), None)
            self.last_plan = replace(
                self.last_plan, target=straight,
                status='OK' if straight is not None else 'NO_STRAIGHT_PATH',
                reason='直路测试：正前方候选检查通过' if straight is not None
                else '直路测试：正前方没有合格候选，停止而不选择侧向通道',
            )
        target = self.last_plan.target
        self.last_feedback = self.motion.feedback(stamp_ns)
        reference = ControlReference(
            stamp_ns=stamp_ns,
            heading_error_rad=None if target is None else target.angle,
            target_speed_mps=self.target_speed_mps if self.control_mode == 'closed_loop' else None,
            stop_requested=target is None,
            reason=self.last_plan.reason,
            reference_id=self.reference_mode,
        )
        self.last_control_input = ControlInput(reference, self.last_feedback)
        self.last_command = self.controller.compute(self.last_control_input)
        self.status = self.last_plan.status if target is None else self.last_command.status
        self.previous_target = target.angle if self.status in ('OK', 'CONTROL_WARMUP') else None
        return self.last_command.steer, self.last_command.speed

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

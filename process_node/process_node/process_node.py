"""激光目标方向跟踪、独立运动反馈、方向 PD 与低速前进 PI。"""

from dataclasses import fields, replace
import math
import os
import signal
import threading
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from robotsdk_msgs.msg import AllSensor, Control

from process_node.control import (
    BaselineControlConfig, BaselineController, ControlInput, ControlReference,
)
from process_node.corridor import CorridorConfig, CorridorTracker, select_cruise_candidate
from process_node.exits import ExitConfig, ExitTracker
from process_node.input_buffer import InputBuffer
from process_node.motion import MotionConfig, MotionEstimator
from process_node.pid_controller import PIDConfig, PIDController
from process_node.perception import PlannerConfig, analyze_scan, analyze_prepared_scan, prepare_scan
from process_node.visualization import draw_laser, draw_steering
from process_node.window_monitor import create_window_monitor
from process_node.turning import TurnBehavior, TurnConfig, TurnResult
from process_node.recovery import LocalRecoveryBehavior, RecoveryConfig


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
        self.declare_parameter("group_label", "midterm-z-turn-v7")
        self.declare_parameter("show_visualization", True)
        self.declare_parameter("control_mode", "closed_loop")
        self.declare_parameter("reference_mode", "corridor")
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
        if self.reference_mode not in ('corridor', 'perception', 'straight'):
            raise ValueError('reference_mode 必须为 corridor、perception 或 straight')
        if self.reference_mode == 'corridor' and self.control_mode != 'closed_loop':
            raise ValueError('corridor 需要 closed_loop；旧基线对照请选择 perception')
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
        corridor_defaults = CorridorConfig()
        for item in fields(corridor_defaults):
            self.declare_parameter('corridor_' + item.name, getattr(corridor_defaults, item.name))
        self.corridor = CorridorTracker(CorridorConfig(**{
            item.name: self.get_parameter('corridor_' + item.name).value
            for item in fields(corridor_defaults)
        }))
        self.last_corridor = None
        exit_defaults = ExitConfig()
        for item in fields(exit_defaults):
            self.declare_parameter('exit_' + item.name, getattr(exit_defaults, item.name))
        self.exits = ExitTracker(ExitConfig(**{
            item.name: self.get_parameter('exit_' + item.name).value
            for item in fields(exit_defaults)
        }), sensor_offset=(self.corridor.config.sensor_x_m, self.corridor.config.sensor_y_m),
            min_finite_ratio=self.config.min_scan_finite_ratio)
        self.last_exits = None
        turn_defaults = TurnConfig()
        for item in fields(turn_defaults):
            self.declare_parameter('turn_' + item.name, getattr(turn_defaults, item.name))
        turn_config = TurnConfig(**{
            item.name: self.get_parameter('turn_' + item.name).value for item in fields(turn_defaults)
        })
        if max(turn_config.speed_mps, turn_config.approach_speed_mps) > pid_config.max_target_speed_mps:
            raise ValueError('turn 速度不能超过 max_target_speed_mps')
        self.turning = TurnBehavior(turn_config, self.config,
                                   sensor_offset=self.exits.sensor_offset, exit_config=self.exits.config)
        self.last_turn = None
        recovery_defaults = RecoveryConfig()
        for item in fields(recovery_defaults):
            self.declare_parameter('recovery_' + item.name, getattr(recovery_defaults, item.name))
        recovery_config = RecoveryConfig(**{
            item.name: self.get_parameter('recovery_' + item.name).value
            for item in fields(recovery_defaults)
        })
        if recovery_config.speed_mps > pid_config.max_target_speed_mps:
            raise ValueError('recovery 速度不能超过 max_target_speed_mps')
        self.recovery = LocalRecoveryBehavior(recovery_config, self.config, turn_config,
                                              self.exits.sensor_offset)
        self.last_recovery = None
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
        self.declare_parameter('input_odom_queue_size', 200)
        self.declare_parameter('visualization_period_s', 0.1)
        self.inputs = InputBuffer(self.get_parameter('input_odom_queue_size').value)
        self.visualization_period = self.get_parameter('visualization_period_s').value
        if not math.isfinite(self.visualization_period) or self.visualization_period <= 0:
            raise ValueError('visualization_period_s 必须是正有限数值')
        self.last_gui_time = -math.inf
        self.last_gui_signature = None
        self.plan_revision = 0
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Control, "control", 10)
        self.subscription = self.create_subscription(
            AllSensor,
            self.get_parameter("sensorTopic").value,
            self.inputs.receive_sensor,
            1,
        )
        self.odometry_subscription = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.inputs.receive_odometry, 200,
        )
        self.last_plan = None
        self.previous_target = None
        self.last_received = None
        self.last_stamp_ns = None
        self.scan_gap_s = None
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

        # ROS 接收线程只唤醒主线程；GTK/OpenCV 的创建、显示和关闭始终留在主线程。
        # 仍使用单调时钟，bag 暂停或播放结束也不会阻止超时和窗口事件处理。
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.ui_timer = self.create_timer(0.05, self.inputs.ready.set, clock=self.steady_clock)
        self.get_logger().info(
            f"[{self.label}] control_mode={self.control_mode}，reference_mode={self.reference_mode}；"
            f"control={self.publisher_.topic_name}，"
            f"odom={self.odometry_subscription.topic_name}，"
            f"require_motion_feedback={self.require_motion_feedback}。"
            f"输入独立接收，GUI={self.gui}，刷新间隔={self.visualization_period:.2f}s。"
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

    def process_pending_inputs(self):
        """主线程先消化已接收的里程计，再处理最新扫描，避免 GUI 拖慢反馈队列。"""
        odometry, sensor, overflow = self.inputs.take()
        if overflow:
            self.motion.reset('ODOM_QUEUE_OVERFLOW', '里程计接收缓冲溢出，重新建立反馈')
            self.previous_target = None
            self.controller.reset()
            self._reset_corridor()
            self.status = 'ODOM_QUEUE_OVERFLOW'
            self.publish_stop()
            self.get_logger().warning('里程计接收缓冲溢出，已停止并重新建立差分历史')
        for item in odometry:
            self.odometry_callback(item.message, received_at=item.received_at)
        if sensor is not None:
            # 使用接收时间判断新鲜度；慢处理不能让一条旧消息重新“保鲜”。
            if time.monotonic() - sensor.received_at > self.timeout:
                self.last_received = sensor.received_at
            else:
                self.laser_scan_callback(sensor.message, received_at=sensor.received_at)

    def odometry_callback(self, msg, received_at=None):
        """运动反馈独立更新，不使用 AllSensor 中的里程计或 IMU 字段。"""
        now = time.monotonic()
        generation = self.motion.generation
        self.motion.update(msg, received_at=now if received_at is None else received_at)
        if self.motion.generation != generation:
            self.previous_target = None
            self.controller.reset()
            self._reset_corridor()
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
            self._reset_corridor(recoverable=self.last_feedback.status in
                                 ('STALE_ODOM', 'UNALIGNED_ODOM', 'WAITING_ODOM'))
            self.publish_stop()
            self.last_stop_time = now
            self.laser_image = None
        self._log_decision()

    def laser_scan_callback(self, msg, received_at=None):
        self.last_received = time.monotonic() if received_at is None else received_at
        steer, speed = self.process_core(msg.urg)
        # 计算若耗时过长，发布前再次检查真实接收时间，不能把已过期的结果发给车辆。
        now = time.monotonic()
        expired = 'STALE_DATA' if now - self.last_received > self.timeout else None
        feedback = self.motion.feedback(self.last_stamp_ns, now=now)
        if expired is None and self.require_motion_feedback and not feedback.valid:
            self.last_feedback = feedback
            expired = feedback.status
        if expired is not None:
            self.status = expired
            self.previous_target = None
            self._reset_corridor(recoverable=expired in
                                 ('STALE_DATA', 'STALE_ODOM', 'UNALIGNED_ODOM'))
            self.last_command = self.controller.stop(expired, '发布前输入/反馈已过期')
            steer, speed = self.last_command.steer, self.last_command.speed
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
                corridor=self.last_corridor,
                exits=self.last_exits,
                turn=self.last_turn,
                recovery=self.last_recovery,
            )

    def _log_decision(self):
        now = time.monotonic()
        log_status = (self.status, self.last_feedback.status,
                      None if self.last_turn is None else (self.last_turn.phase, self.last_turn.status),
                      None if self.last_recovery is None else
                      (self.last_recovery.phase, self.last_recovery.status))
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
        if self.last_turn is not None:
            turn = self.last_turn
            control_text += (f' | behavior={turn.phase} turn_status={turn.status}'
                             f' side={turn.side} chain={turn.chain_stage}'
                             f' entry_basis={turn.entry_basis}'
                             f' locked_exit={turn.exit_id} curvature={turn.curvature_per_m:+.3f}/m'
                             f' ({turn.reason})')
        if self.last_recovery is not None:
            recovery = self.last_recovery
            control_text += (f' | recovery={recovery.phase}/{recovery.status}'
                             f' curvature={recovery.curvature_per_m:+.3f}/m'
                             f' cause=({recovery.cause}) reason=({recovery.reason})')
        result = self.last_plan
        if self.last_corridor is not None:
            estimate = self.last_corridor
            control_text += (f' | corridor={estimate.status} confidence={estimate.confidence:.2f}'
                             f' fitted_walls={len(estimate.observed_walls)}')
            if self.scan_gap_s is not None:
                control_text += f' scan_dt={self.scan_gap_s:.3f}s'
            if estimate.heading_rad is not None:
                control_text += (
                    f' axis={math.degrees(estimate.heading_rad):+.2f}deg'
                    f' center_left={estimate.lateral_error_m:+.3f}m width={estimate.width_m:.3f}m'
                    f' age={estimate.measurement_age_s:.3f}s pair_age={estimate.pair_age_s:.3f}s'
                    f' speed_scale={estimate.speed_scale:.2f}'
                )
            control_text += f' ({estimate.reason})'
        if self.last_exits is not None:
            exits = self.last_exits
            descriptions = ','.join(
                f'#{t.track_id}:{t.side}:{t.state}@({t.observation.center_xy[0]:.2f},'
                f'{t.observation.center_xy[1]:.2f})'
                f'/{math.degrees(t.observation.heading_rad):+.0f}deg'
                f'/kind={t.observation.role}'
                f'/w={t.observation.width_m:.2f}/deep={t.observation.depth_visible_ratio:.0%}'
                f'/age={t.age_s:.2f}'
                for t in exits.tracks)
            control_text += (f' | exits={exits.status} suggestion={exits.recommendation}'
                             f' preferred={exits.preferred_id} [{descriptions}] (diagnostic only)')
        if result is None or self.status == "STALE_DATA":
            self.get_logger().info(
                f"[{self.label}] {self.status}：等待有效传感器输入，输出停止值 | {motion_text}"
                f" | {control_text}"
            )
            return
        data = result.scan
        target = result.target
        active_angle = target.angle if target is not None else (
            self.last_turn.heading_rad if self.last_turn is not None and self.last_turn.active else None)
        if self.last_recovery is not None and self.last_recovery.active:
            active_angle = self.last_recovery.heading_rad
        direction = "--" if active_angle is None else f"{math.degrees(active_angle):+.2f}deg"
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
            f" | input_skipped_scans={self.inputs.skipped_scans}"
            f" | perception={result.status}: {result.reason}"
            f" | {control_text}"
        )

    def _tick(self):
        now = time.monotonic()
        if self.last_received is None or now - self.last_received > self.timeout:
            self.status = "WAITING" if self.last_received is None else "STALE_DATA"
            self.previous_target = None
            self._reset_corridor(recoverable=True)
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
                self._reset_corridor(recoverable=self.last_feedback.status in
                                     ('STALE_ODOM', 'UNALIGNED_ODOM', 'WAITING_ODOM'))
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
            signature = (self.plan_revision, self.status, self.steer_value, self.speed_value)
            # X11/VS Code 转发大图像可能很慢。状态不变不重传整幅图，
            # 新帧最多按配置频率刷新，但每次 tick 都继续处理按键和窗口关闭。
            if (signature != self.last_gui_signature
                    and (now - self.last_gui_time >= self.visualization_period
                         or self.last_gui_signature is None
                         or self.status != self.last_gui_signature[1])):
                image = self.camera_image.copy()
                target = self.previous_target if self.status == "OK" else None
                draw_steering(image, self.steer_value, self.steer_center, self.steer_limit,
                              self.label, self.status, target)
                cv2.imshow(self.CAMERA_WINDOW, image)
                laser = self.laser_image
                if laser is None:
                    laser = draw_laser(
                        self.last_plan, self.config, self.status, self.steer_value,
                        self.speed_value, self.label, command=self.last_command,
                        corridor=self.last_corridor,
                        exits=self.last_exits,
                        turn=self.last_turn,
                        recovery=self.last_recovery,
                    )
                cv2.imshow(self.LASER_WINDOW, laser)
                self.last_gui_signature, self.last_gui_time = signature, now
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

    def _reset_corridor(self, recoverable=False, reason=None):
        self.corridor.reset()
        self.last_corridor = None
        self.exits.reset()
        self.last_exits = None
        if reason is None:
            reason = (f'输入/反馈异常({self.motion.status}/{self.last_feedback.status})，'
                      + ('停车重新确认' if recoverable else '禁止自动沿用历史参考'))
        self.turning.invalidate(reason, recoverable=recoverable)
        self.recovery.interrupt(reason, hard=not recoverable)
        self.last_turn = None
        self.last_recovery = None

    def _corridor_plan(self, urg, stamp_ns):
        """先生成运动补偿后的巡航参考，再由原感知检查实际可执行候选。"""
        try:
            data = prepare_scan(urg, self.config)
        except (ValueError, TypeError, OverflowError) as error:
            self._reset_corridor(reason=f'扫描元数据无效：{error}')
            return analyze_scan(urg, self.config)
        if (data.too_close.any() or not data.valid.any()
                or data.valid_ratio < self.config.min_scan_finite_ratio):
            self._reset_corridor(reason='雷达过近、没有真实有限回波或真实回波比例不足')
            return analyze_prepared_scan(data, self.config)
        estimate = self.corridor.update(data, self.last_feedback, stamp_ns)
        self.last_corridor = estimate
        # 精确参考角也必须通过覆盖、无回波范围和完整车体扫掠，不能强行下发。
        extra = (estimate.target_angle_rad,) if estimate.usable else ()
        result = analyze_prepared_scan(data, self.config, extra_headings=extra)
        if result.status != 'OK':
            return result
        if not estimate.usable:
            status = estimate.status if self.last_feedback.valid else self.last_feedback.status
            return replace(result, target=None, status=status, reason=estimate.reason)
        target = select_cruise_candidate(result.candidates, estimate, self.corridor.config)
        return replace(result, target=target, status='OK' if target is not None else 'CRUISE_BLOCKED',
                       reason='巡航参考附近的候选通过覆盖和车体检查' if target is not None
                       else '巡航参考附近没有合格候选，停止并等待后续行为')

    def process_core(self, urg):
        """输入 LaserScan，返回 (steer, speed)，保持课程模板接口。"""
        self.plan_revision += 1
        stamp = urg.header.stamp
        stamp_ns = stamp.sec * 10**9 + stamp.nanosec
        self.scan_gap_s = None if self.last_stamp_ns is None else (stamp_ns-self.last_stamp_ns)*1e-9
        if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
            self.previous_target = None
            self.controller.reset()
            self._reset_corridor(reason='激光时间回跳，不能自动沿用历史位姿和路径')
            # 新一轮 odom 可能先于激光到达，只有仍留着旧时间段的反馈时才清空。
            if (self.motion.state is not None and self.motion.state.stamp_ns
                    > stamp_ns + round(self.motion.config.scan_tolerance_s * 1e9)):
                self.motion.reset('ODOM_TIME_REWIND', '激光时间回跳，等待同一时间段的新反馈')
        self.last_stamp_ns = stamp_ns
        self.last_feedback = self.motion.feedback(stamp_ns)
        if self.reference_mode == 'corridor':
            self.last_plan = self._corridor_plan(urg, stamp_ns)
        else:
            self.last_corridor = None
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
        # 本帧出口先于行为仲裁更新。转弯使用自己的路径门控，不依赖旧巡航目标可用。
        self.last_exits = self.exits.update(
            self.last_plan.scan, self.last_feedback, stamp_ns, self.last_corridor)
        target = self.last_plan.target
        heading = None if target is None else target.angle
        speed_scale = self.last_corridor.speed_scale \
            if self.reference_mode == 'corridor' and self.last_corridor is not None else 1.0
        requested_speed = self.target_speed_mps * speed_scale
        cruise_angle, cruise_status = heading, self.last_plan.status
        reference_id = self.reference_mode
        reason = self.last_plan.reason
        if self.reference_mode == 'corridor':
            if self.recovery.active and self.turning.phase == 'CRUISE':
                # 恢复过程中不另开新的路口任务；原已锁定出口仍由 turning 保留。
                self.last_turn = TurnResult('CRUISE', 'OK', '局部恢复期间暂停新转弯选择')
            else:
                self.last_turn = self.turning.update(
                    self.last_plan.scan, self.last_feedback, stamp_ns, self.last_corridor,
                    self.last_exits, cruise_angle=heading)
            turn = self.last_turn
            if turn.reset_corridor:
                # 仅重置拟合器；不能调用 _reset_corridor 取消正在执行的转弯。
                self.corridor.reset()
            if turn.active:
                heading, reason = turn.heading_rad, turn.reason
                requested_speed = min(self.target_speed_mps, turn.speed_mps)
                reference_id = f'{turn.side.lower()}_turn:{turn.exit_id}:leg{turn.chain_stage}'
                # 曲线目标不伪装成经过直线筛选的 Candidate，图形使用独立路径字段。
                self.last_plan = replace(self.last_plan, target=None,
                                         status=turn.status, reason=reason)
            # 正常行为失败时才启用恢复；其候选独立检查曲线整车，不复用直线候选冒充转弯。
            self.last_recovery = self.recovery.update(
                self.last_plan.scan, self.last_feedback, stamp_ns, self.last_corridor,
                cruise_angle, cruise_status, turn, self.turning.recovery_goal,
                received_at=self.last_received)
            recovery = self.last_recovery
            if recovery.active:
                heading, reason = recovery.heading_rad, recovery.reason
                requested_speed = min(self.target_speed_mps, recovery.speed_mps)
                reference_id = 'local_recovery'
                self.last_plan = replace(self.last_plan, target=None,
                                         status=recovery.status, reason=reason)
                if recovery.handoff:
                    self.last_turn = self.turning.finish_recovery()
                elif recovery.phase == 'MOVE' and turn.active:
                    # 开始局部修正后，不再追赶旧路径；保留出口意图与首次异常原因。
                    self.turning.invalidate('旧转弯路径交给局部恢复重新检查', recoverable=True)
        else:
            self.last_turn = None
            self.last_recovery = None
        reference = ControlReference(
            stamp_ns=stamp_ns,
            heading_error_rad=heading,
            target_speed_mps=requested_speed if self.control_mode == 'closed_loop' else None,
            stop_requested=heading is None,
            reason=reason,
            reference_id=reference_id,
        )
        self.last_control_input = ControlInput(reference, self.last_feedback)
        self.last_command = self.controller.compute(self.last_control_input)
        self.status = self.last_plan.status if heading is None else self.last_command.status
        self.previous_target = heading if self.status in ('OK', 'CONTROL_WARMUP') else None
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
    executor = None
    receiver = None
    receiver_errors = []
    try:
        node = LaserWindowAnalyzer()
        executor = SingleThreadedExecutor(context=node.context)
        executor.add_node(node)

        def receive_messages():
            try:
                executor.spin()
            except ExternalShutdownException:
                pass
            except Exception as error:
                receiver_errors.append(error)
                node.inputs.ready.set()

        receiver = threading.Thread(target=receive_messages, name='ros-input', daemon=True)
        receiver.start()
        next_tick = time.monotonic()
        while rclpy.ok() and not node.exit_requested and not stop_requested:
            node.inputs.ready.wait(timeout=0.01)
            node.inputs.ready.clear()
            if receiver_errors:
                raise RuntimeError('ROS 接收线程异常，停止控制') from receiver_errors[0]
            node.process_pending_inputs()
            if time.monotonic() >= next_tick:
                node._tick()
                next_tick = time.monotonic() + 0.05
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_stop()
            if executor is not None:
                executor.shutdown()
            if receiver is not None:
                receiver.join()
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

"""统一控制接口与当前基线适配器；PD/PI 后续沿用相同输入和输出。"""

from dataclasses import dataclass
import math
from typing import Optional

from process_node.motion import MotionFeedback


@dataclass(frozen=True)
class ControlReference:
    """感知/行为层给出的目标；物理目标与底层动力控制量分开。"""

    stamp_ns: int  # 目标对应的激光消息时间，单位 ns；后续控制器以此计算 dt
    heading_error_rad: Optional[float]  # 目标相对当前车头的方向误差，左为正，单位 rad
    target_speed_mps: Optional[float] = None  # 物理目标速度，单位 m/s；基线尚未使用
    stop_requested: bool = False  # 行为或保护层要求停车时优先处理
    reason: str = ''  # 本次目标或停止请求的来源说明


@dataclass(frozen=True)
class ControlInput:
    """后续控制器的统一输入；反馈包含时间对齐结果及有效性。"""

    reference: ControlReference  # 感知/行为层目标
    feedback: MotionFeedback  # 独立里程计反馈，不能只读取数值而忽略 valid


@dataclass(frozen=True)
class ControlCommand:
    """算法输出；由 ROS 集成层转换为课程的 Control 消息。"""

    steer: int  # 底层转向控制量，不等于车轮角度
    speed: int  # 底层动力控制量，不等于 m/s
    status: str  # OK、保护停止原因或接口不支持等状态
    reason: str  # 输出依据


@dataclass(frozen=True)
class BaselineControlConfig:
    """保留当前方向比例映射和固定动力输出，默认要求反馈有效。"""

    steer_center: int = 0  # 车辆直行转向中位，需标定
    steer_limit: int = 600  # 相对中位的最大控制幅度
    steer_gain_per_rad: float = -300.0  # 弧度到转向控制量的比例增益
    cruise_speed: int = -100  # 基线固定动力控制量，需标定
    stop_speed: int = 0  # 停止动力控制量，需标定
    require_motion_feedback: bool = True  # 关闭仅用于离线感知观察，不用于闭环运行

    def __post_init__(self):
        if (not 0 < self.steer_limit <= 32767
                or abs(self.steer_center) + self.steer_limit > 32767
                or any(not -32768 <= value <= 32767
                       for value in (self.cruise_speed, self.stop_speed))
                or not math.isfinite(self.steer_gain_per_rad)):
            raise ValueError('控制量需符合 int16 范围，转向增益需为有限数值')


class BaselineController:
    """接口落地用的基线控制器；当前尚未利用速度反馈调节动力。"""

    def __init__(self, config=BaselineControlConfig()):
        self.config = config

    def stop(self, status, reason):
        return ControlCommand(self.config.steer_center, self.config.stop_speed, status, reason)

    def compute(self, control_input):
        reference, feedback = control_input.reference, control_input.feedback
        if reference.stop_requested:
            return self.stop('STOP_REQUESTED', reference.reason)
        if not isinstance(reference.stamp_ns, int) or reference.stamp_ns < 0:
            return self.stop('INVALID_REFERENCE', '目标消息时间必须为非负整数纳秒')
        if reference.heading_error_rad is None or not math.isfinite(reference.heading_error_rad):
            return self.stop('INVALID_REFERENCE', '没有有效的目标方向')
        if self.config.require_motion_feedback and not feedback.valid:
            return self.stop(feedback.status, feedback.reason)
        if reference.target_speed_mps is not None:
            # 不允许把 m/s 悄悄当成固定动力输出，后续速度控制器将实现这个字段。
            return self.stop('UNSUPPORTED_REFERENCE', '当前基线不支持物理目标速度，尚未实现 PI')
        delta = round(max(-self.config.steer_limit, min(self.config.steer_limit,
                    self.config.steer_gain_per_rad * reference.heading_error_rad)))
        return ControlCommand(self.config.steer_center + delta, self.config.cruise_speed,
                              'OK', '方向比例映射与固定动力基线；尚未运行 PD/PI')

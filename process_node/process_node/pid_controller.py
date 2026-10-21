"""方向 PD + 速度 PI；无 ROS 依赖，所有时间计算来自输入消息。"""

from dataclasses import dataclass
import math

from process_node.control import ControlCommand, ControlDiagnostics
from process_node.motion import wrap_angle


def clip(value, low, high):
    return max(low, min(high, value))


def slew(target, previous, rate, dt):
    """限制正常控制输出每秒的变化量；紧急停止不经过此函数。"""
    return clip(target, previous - rate * dt, previous + rate * dt)


@dataclass(frozen=True)
class PIDConfig:
    """默认值仅供离线调试，不代表已在课程车辆上整定。"""

    steer_center: int = 0  # 直行中位（控制单位），需实车标定
    steer_limit: int = 600  # 相对中位的转向幅值上限（控制单位）
    stop_speed: int = 0  # 停止动力值（控制单位），需实车标定
    steer_sign: int = -1  # 正方向误差对应的底层转向符号，只能为 ±1
    steer_kp: float = 300.0  # 方向 P 增益（控制单位/rad）
    steer_kd: float = 30.0  # 方向 D 增益（控制单位·s/rad）
    steer_derivative_tau_s: float = 0.15  # 方向误差微分的低通时间常数（s），0 禁用滤波
    steer_jump_reset_deg: float = 20.0  # 相邻目标误差变化超过此角度时撤去 D 项（度）
    steer_rate_limit: float = 300.0  # 正常转向控制量的最大变化率（控制单位/s）
    speed_sign: int = -1  # 正的前进动力幅度映射到底层的符号，只能为 ±1
    speed_kp: float = 100.0  # 速度 P 增益（控制单位/(m/s)）
    speed_ki: float = 30.0  # 速度 I 增益（控制单位/((m/s)·s)）
    speed_feedforward_gain: float = 200.0  # 目标速度的线性前馈系数（控制单位/(m/s)）
    speed_feedforward_offset: float = 0.0  # 正目标速度时增加的前馈偏置（控制单位），标定死区后再设
    speed_effort_limit: float = 120.0  # 前进动力幅度上限（控制单位），本版不输出倒车动力
    speed_integral_limit: float = 30.0  # 积分项的绝对上限（控制单位，含 Ki）
    speed_rate_limit: float = 80.0  # 正常动力控制量最大变化率（控制单位/s）
    target_accel_mps2: float = 0.2  # 内部目标速度增加的最大速率（m/s²）
    target_decel_mps2: float = 0.4  # 正目标速度下调的最大速率（m/s²）；请求零速直接停车
    max_target_speed_mps: float = 0.5  # 接口接受的最大前进目标速度（m/s）
    control_min_dt_s: float = 0.01  # 过短的控制间隔累积后再计算（s）
    control_max_dt_s: float = 0.5  # 超过此消息间隔停止并重建历史（s）

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value):
                raise ValueError(f'{name} 必须为有限数值')
            if name not in ('steer_center', 'stop_speed', 'steer_sign', 'speed_sign') and value < 0:
                raise ValueError(f'{name} 不能为负')
        if self.steer_sign not in (-1, 1) or self.speed_sign not in (-1, 1):
            raise ValueError('steer_sign、speed_sign 只能为 ±1')
        if (not 0 < self.steer_limit <= 32767
                or abs(self.steer_center) + self.steer_limit > 32767
                or not -32768 <= self.stop_speed <= 32767
                or not -32768 <= self.stop_speed + self.speed_sign * self.speed_effort_limit <= 32767):
            raise ValueError('转向和动力输出范围必须符合 int16')
        if not 0 < self.control_min_dt_s < self.control_max_dt_s:
            raise ValueError('需要 0 < control_min_dt_s < control_max_dt_s')
        if min(self.steer_rate_limit, self.speed_rate_limit, self.speed_effort_limit,
               self.target_accel_mps2, self.target_decel_mps2, self.max_target_speed_mps) <= 0:
            raise ValueError('输出变化率、动力上限和目标速度/加减速上限必须为正')
        if not 0 < self.steer_jump_reset_deg <= 180:
            raise ValueError('steer_jump_reset_deg 必须在 (0, 180] 内')


class PIDController:
    """前进速度 PI 与目标方向 PD 的组合，停止请求始终覆盖普通输出。"""

    def __init__(self, config=PIDConfig()):
        self.config = config
        self.reset()

    def reset(self):
        """反馈/时间/行为状态失效时调用，下一帧重新建立时间基准。"""
        self._stamp_ns = None
        self._reference_id = None
        self._error = None
        self._derivative = 0.0
        self._integral = 0.0
        self._target_speed = 0.0
        self._steer_offset = 0.0
        self._effort = 0.0
        self._last_command = ControlCommand(
            self.config.steer_center, self.config.stop_speed,
            'CONTROL_WARMUP', '等待相邻控制消息建立时间差',
        )

    def stop(self, status, reason):
        """停止直接回中位/停止动力值，绕过正常输出变化率限制并清空历史。"""
        self.reset()
        self._last_command = ControlCommand(
            self.config.steer_center, self.config.stop_speed, status, reason,
        )
        return self._last_command

    def compute(self, control_input):
        reference, feedback = control_input.reference, control_input.feedback
        cfg = self.config
        if reference.stop_requested:
            return self.stop('STOP_REQUESTED', reference.reason)
        if (not isinstance(reference.stamp_ns, int) or reference.stamp_ns < 0
                or reference.heading_error_rad is None
                or not math.isfinite(reference.heading_error_rad)
                or reference.target_speed_mps is None
                or not math.isfinite(reference.target_speed_mps)
                or not 0 <= reference.target_speed_mps <= cfg.max_target_speed_mps
                or not reference.reference_id):
            return self.stop('INVALID_REFERENCE', '需要有效消息时间、方向及范围内的非负物理目标速度')
        if reference.target_speed_mps == 0:
            return self.stop('STOP_REQUESTED', '目标速度为零，清空历史并停止')
        # 闭环运行始终依赖有效反馈，不允许通过离线观察开关绕过。
        if not feedback.valid:
            return self.stop(feedback.status, feedback.reason)
        measured_speed = feedback.state.speed_mps
        if not math.isfinite(measured_speed):
            return self.stop('INVALID_FEEDBACK', '速度反馈不是有限数值')
        error = wrap_angle(reference.heading_error_rad)
        if self._stamp_ns is None:
            self._stamp_ns = reference.stamp_ns
            self._reference_id = reference.reference_id
            self._error = error
            self._last_command = ControlCommand(
                cfg.steer_center, cfg.stop_speed, 'CONTROL_WARMUP',
                '有效输入已恢复，等待下一控制消息建立时间差',
            )
            return self._last_command

        dt = (reference.stamp_ns - self._stamp_ns) * 1e-9
        if dt <= 0:
            return self.stop('CONTROL_TIME_INVALID', '控制消息时间重复或回跳，停止并清空历史')
        if dt > cfg.control_max_dt_s:
            return self.stop('CONTROL_TIME_GAP', '控制消息间隔过长，停止并清空历史')
        if dt < cfg.control_min_dt_s:
            # 不更新起点、积分和微分，直到实际消息间隔足够；保护检查已在前面完成。
            return self._last_command

        switched = reference.reference_id != self._reference_id
        if switched:
            self._integral = 0.0
        change = 0.0 if self._error is None else wrap_angle(error - self._error)
        reset_derivative = switched or abs(change) > math.radians(cfg.steer_jump_reset_deg)
        if reset_derivative:
            self._derivative = 0.0
        else:
            raw_derivative = change / dt
            alpha = 1.0 if cfg.steer_derivative_tau_s == 0 else \
                -math.expm1(-dt / cfg.steer_derivative_tau_s)
            self._derivative += alpha * (raw_derivative - self._derivative)
        steer_p = cfg.steer_sign * cfg.steer_kp * error
        steer_d = cfg.steer_sign * cfg.steer_kd * self._derivative
        raw_steer = steer_p + steer_d
        bounded_steer = clip(raw_steer, -cfg.steer_limit, cfg.steer_limit)
        steer = slew(bounded_steer, self._steer_offset, cfg.steer_rate_limit, dt)

        # 从零平滑建立物理目标速度；零速请求已在前面直接停车。
        speed_rate = cfg.target_accel_mps2 if reference.target_speed_mps > self._target_speed \
            else cfg.target_decel_mps2
        self._target_speed = slew(reference.target_speed_mps, self._target_speed, speed_rate, dt)
        speed_error = self._target_speed - measured_speed
        feedforward = cfg.speed_feedforward_offset + cfg.speed_feedforward_gain * self._target_speed
        speed_p = cfg.speed_kp * speed_error
        increment = cfg.speed_ki * speed_error * dt
        candidate_integral = clip(self._integral + increment,
                                  -cfg.speed_integral_limit, cfg.speed_integral_limit)
        candidate_raw = feedforward + speed_p + candidate_integral
        candidate_applied = slew(clip(candidate_raw, 0.0, cfg.speed_effort_limit),
                                 self._effort, cfg.speed_rate_limit, dt)
        # 条件积分：幅值饱和或变化率限制时，拒绝让饱和继续加重的积分增量；
        # 反方向的积分仍允许进入，以便退出饱和。限幅和变化率限制均计入此检查。
        integral_frozen = ((candidate_raw > candidate_applied + 1e-9 and increment > 0)
                           or (candidate_raw < candidate_applied - 1e-9 and increment < 0))
        if not integral_frozen:
            self._integral = candidate_integral
        raw_effort = feedforward + speed_p + self._integral
        bounded_effort = clip(raw_effort, 0.0, cfg.speed_effort_limit)
        effort = slew(bounded_effort, self._effort, cfg.speed_rate_limit, dt)

        diagnostics = ControlDiagnostics(
            dt_s=dt, requested_speed_mps=reference.target_speed_mps,
            target_speed_mps=self._target_speed, measured_speed_mps=measured_speed,
            speed_error_mps=speed_error, heading_error_rad=error,
            heading_rate_rps=self._derivative, steer_p=steer_p, steer_d=steer_d,
            speed_feedforward=feedforward, speed_p=speed_p, speed_i=self._integral,
            steer_saturated=abs(raw_steer) > cfg.steer_limit,
            speed_saturated=raw_effort < 0 or raw_effort > cfg.speed_effort_limit,
            steer_rate_limited=abs(steer - bounded_steer) > 1e-9,
            speed_rate_limited=abs(effort - bounded_effort) > 1e-9,
            integral_frozen=integral_frozen, derivative_reset=reset_derivative,
        )
        self._stamp_ns = reference.stamp_ns
        self._reference_id = reference.reference_id
        self._error = error
        self._steer_offset, self._effort = steer, effort
        self._last_command = ControlCommand(
            round(cfg.steer_center + steer), round(cfg.stop_speed + cfg.speed_sign * effort),
            'OK', '方向 PD 与前进速度 PI；动力前馈和增益待实车标定', diagnostics,
        )
        return self._last_command

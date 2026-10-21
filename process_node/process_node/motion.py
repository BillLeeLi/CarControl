"""独立里程计反馈：只读位置、姿态和消息时间，不使用有问题的 twist.linear。"""

from collections import deque
from dataclasses import dataclass
import math
import time
from typing import Optional


def wrap_angle(angle):
    """相邻航向差取最短角，避免 +pi/-pi 接缝造成虚假的整圈旋转。"""
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class MotionConfig:
    """时间、滤波及异常阈值；节点将这些字段注册为 odom_ 开头的参数。"""

    min_dt_s: float = 0.01  # 差分最短间隔，太密时保留起点，累积到足够间隔再估速
    max_dt_s: float = 0.5  # 超过此消息间隔重新建立差分起点，不跨长缺测段估速
    filter_tau_s: float = 0.15  # 速度和角速度一阶低通时间常数；0 表示不滤波
    max_speed_mps: float = 3.0  # 位移跳变检查的速度上限，需结合实车核对
    max_yaw_rate_rps: float = 6.0  # 航向跳变检查的角速度上限，单位 rad/s
    timeout_s: float = 0.5  # 有效估计的接收超时，使用单调时钟秒数
    scan_tolerance_s: float = 0.15  # 雷达与关联反馈允许的最大消息时间差
    history_s: float = 2.0  # 按消息时间保留的反馈历史长度
    frame_id: str = 'odom'  # 位置和航向所在坐标系；本模块不执行 TF 变换
    child_frame_id: str = 'base_link'  # 被估计的车体坐标系，+x 为车头方向

    def __post_init__(self):
        for name, value in vars(self).items():
            if name in ('frame_id', 'child_frame_id'):
                if not value:
                    raise ValueError(f'{name} 不能为空')
            elif not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} 必须是非负有限数值')
        if not 0 < self.min_dt_s < self.max_dt_s:
            raise ValueError('需要 0 < odom_min_dt_s < odom_max_dt_s')
        if min(self.max_speed_mps, self.max_yaw_rate_rps, self.timeout_s,
               self.scan_tolerance_s) <= 0:
            raise ValueError('运动上限、接收超时和时间对齐容差必须大于 0')
        if self.history_s < self.max_dt_s + self.scan_tolerance_s:
            raise ValueError('反馈历史长度必须覆盖最大差分间隔及时间对齐容差')


@dataclass(frozen=True)
class MotionState:
    """一次有效差分得到的平面运动状态；距离和航向累计量在重置后重新计数。"""

    stamp_ns: int  # 本次估计终点的里程计消息时间，单位 ns
    dt_s: float  # 参与本次位置/航向差分的实际消息时间间隔，单位 s
    x_m: float  # odom 坐标系内的位置 x，单位 m
    y_m: float  # odom 坐标系内的位置 y，单位 m
    yaw_rad: float  # 当前航向，归一化到 [-pi, pi)，单位 rad
    yaw_unwrapped_rad: float  # 连续展开的航向，跨过 pi 时不突跳，单位 rad
    local_x_m: float  # 相对本次起点的位置，沿起点车头方向为正，单位 m
    local_y_m: float  # 相对本次起点的位置，沿起点车体左侧为正，单位 m
    heading_change_rad: float  # 自本次起点累计的有符号航向变化，单位 rad
    distance_m: float  # 累加各段位移长度，前进/倒退都增加，单位 m
    signed_distance_m: float  # 累加沿车头方向投影的位移，倒退减少，单位 m
    raw_speed_mps: float  # 本次位移投影 / dt，未滤波的有符号速度，单位 m/s
    speed_mps: float  # 一阶低通后的有符号前进速度，单位 m/s
    yaw_rate_rps: float  # 由航向差分并滤波得到的角速度，左转为正，单位 rad/s


@dataclass(frozen=True)
class MotionFeedback:
    """反馈状态与可用性一起传递，避免把旧值或初始化时的零误当作有效速度。"""

    state: Optional[MotionState]  # 可供诊断的状态；valid 为假时不能用于闭环控制
    status: str  # OK 或等待、超时、时间不匹配、输入异常等状态
    reason: str  # 状态的中文解释
    skew_s: Optional[float] = None  # 反馈时间减雷达时间，正值表示反馈稍晚

    @property
    def valid(self):
        return self.status == 'OK' and self.state is not None


class MotionEstimator:
    """无 ROS 运行时依赖；接收具有 Odometry 字段的对象，便于独立测试。"""

    def __init__(self, config=MotionConfig()):
        self.config = config
        self.generation = 0  # 每次重置递增，集成层据此清除旧目标/后续控制器历史
        self.reset()

    def reset(self, status='WAITING_ODOM', reason='等待独立里程计输入'):
        self.generation += 1
        self.status, self.reason = status, reason
        self.state = None
        self.last_received = None
        self._previous = None
        self._last_stamp_ns = None
        self._origin = None
        self._history = deque()

    def _seed(self, sample, status, reason):
        """用当前点建立新起点；至少再收到一个有效间隔后，才有速度反馈。"""
        self.reset(status, reason)
        self._previous = sample
        self._last_stamp_ns = sample[0]
        self._origin = sample[1:]

    def update(self, odom, received_at=None):
        """用消息时间计算运动，用接收时间检查是否持续收到有效估计。"""
        now = time.monotonic() if received_at is None else received_at
        try:
            stamp = odom.header.stamp
            if stamp.sec < 0 or not 0 <= stamp.nanosec < 10**9:
                raise ValueError('里程计时间戳不合法')
            stamp_ns = stamp.sec * 10**9 + stamp.nanosec
            if (odom.header.frame_id != self.config.frame_id
                    or odom.child_frame_id != self.config.child_frame_id):
                raise ValueError('里程计坐标系不匹配，需核对 odom_frame_id/odom_child_frame_id')
            pose = odom.pose.pose
            x, y = pose.position.x, pose.position.y
            q = pose.orientation
            values = (x, y, q.x, q.y, q.z, q.w, now)
            if not all(math.isfinite(v) for v in values):
                raise ValueError('里程计位置、姿态或接收时间包含非有限数值')
            norm = math.hypot(q.x, q.y, q.z, q.w)
            if not math.isfinite(norm) or norm < 1e-6:
                raise ValueError('里程计姿态四元数长度不合法')
            qx, qy, qz, qw = (v / norm for v in (q.x, q.y, q.z, q.w))
            yaw = math.atan2(2 * (qw * qz + qx * qy),
                             1 - 2 * (qy * qy + qz * qz))
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            self.reset('INVALID_ODOM', str(error))
            return self.feedback(now=now)

        sample = (stamp_ns, x, y, yaw)
        if self._previous is None:
            self._seed(sample, 'INITIALIZING_ODOM', '已收到起点，等待足够间隔的位置差分')
            return self.feedback(now=now)
        if stamp_ns < self._last_stamp_ns:
            self._seed(sample, 'ODOM_TIME_REWIND', '里程计时间回跳，已清除轨迹和差分历史')
            return self.feedback(now=now)
        if stamp_ns == self._last_stamp_ns:
            # 重复消息不能延长有效反馈的寿命，也不改变差分起点。
            self.status, self.reason = 'DUPLICATE_ODOM', '重复的里程计时间戳，等待新数据'
            return self.feedback(now=now)
        self._last_stamp_ns = stamp_ns
        previous_ns, px, py, previous_yaw = self._previous
        dt = (stamp_ns - previous_ns) * 1e-9
        if dt < self.config.min_dt_s:
            return self.feedback(now=now)
        if dt > self.config.max_dt_s:
            self._seed(sample, 'ODOM_TIME_GAP', '里程计间隔过长，重新建立差分起点')
            return self.feedback(now=now)

        dx, dy = x - px, y - py
        dyaw = wrap_angle(yaw - previous_yaw)
        distance = math.hypot(dx, dy)
        if (distance / dt > self.config.max_speed_mps
                or abs(dyaw) / dt > self.config.max_yaw_rate_rps):
            self._seed(sample, 'ODOM_POSE_JUMP', '位姿变化超过配置运动上限，重新建立起点')
            return self.feedback(now=now)
        # 位移投影到这一小段的中间车头方向：前进为正、倒退为负。
        middle_yaw = previous_yaw + dyaw / 2
        signed_distance = dx * math.cos(middle_yaw) + dy * math.sin(middle_yaw)
        raw_speed, raw_yaw_rate = signed_distance / dt, dyaw / dt
        old = self.state
        alpha = 1.0 if self.config.filter_tau_s == 0 else \
            -math.expm1(-dt / self.config.filter_tau_s)
        speed = raw_speed if old is None else old.speed_mps + alpha * (raw_speed - old.speed_mps)
        yaw_rate = raw_yaw_rate if old is None else \
            old.yaw_rate_rps + alpha * (raw_yaw_rate - old.yaw_rate_rps)
        ox, oy, origin_yaw = self._origin
        local_dx, local_dy = x - ox, y - oy
        heading = dyaw + (0.0 if old is None else old.heading_change_rad)
        self.state = MotionState(
            stamp_ns=stamp_ns, dt_s=dt, x_m=x, y_m=y, yaw_rad=yaw,
            yaw_unwrapped_rad=origin_yaw + heading,
            local_x_m=local_dx * math.cos(origin_yaw) + local_dy * math.sin(origin_yaw),
            local_y_m=-local_dx * math.sin(origin_yaw) + local_dy * math.cos(origin_yaw),
            heading_change_rad=heading,
            distance_m=distance + (0.0 if old is None else old.distance_m),
            signed_distance_m=signed_distance + (0.0 if old is None else old.signed_distance_m),
            raw_speed_mps=raw_speed, speed_mps=speed, yaw_rate_rps=yaw_rate,
        )
        self._previous = sample
        self.last_received = now
        self.status, self.reason = 'OK', '独立里程计位置/姿态差分有效'
        self._history.append(self.state)
        cutoff = stamp_ns - round(self.config.history_s * 1e9)
        while self._history[0].stamp_ns < cutoff:
            self._history.popleft()
        return self.feedback(now=now)

    def feedback(self, stamp_ns=None, now=None):
        """从有限历史选消息时间最接近的状态；不外推，也不等待未来消息。"""
        now = time.monotonic() if now is None else now
        if self.status != 'OK' or self.state is None:
            return MotionFeedback(self.state, self.status, self.reason)
        if now - self.last_received > self.config.timeout_s:
            return MotionFeedback(self.state, 'STALE_ODOM', '超过有效里程计接收超时')
        if stamp_ns is None:
            return MotionFeedback(self.state, 'OK', self.reason)
        state = min(self._history, key=lambda item: abs(item.stamp_ns - stamp_ns))
        skew = (state.stamp_ns - stamp_ns) * 1e-9
        if abs(skew) > self.config.scan_tolerance_s:
            return MotionFeedback(state, 'UNALIGNED_ODOM', '雷达与反馈的消息时间差过大', skew)
        return MotionFeedback(state, 'OK', self.reason, skew)

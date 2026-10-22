"""不依赖 ROS 的激光感知基线：真实角度、观测可信度和局部通道选择。"""

from dataclasses import dataclass, field
import math

import numpy as np


def wrap_angle(angle):
    """把弧度归一化到 [-pi, pi)，让扫描首尾的前向数据连在一起。"""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


@dataclass(frozen=True)
class PlannerConfig:
    """这些几何默认值只供离线分析，上车前必须按实际车辆测量。"""

    sensor_yaw_offset_deg: float = 0.0  # 雷达到车体坐标的安装偏角（度），正值逆时针
    vehicle_width_m: float = 0.35  # 车体总宽（米），用于计算车体包络圆盘
    front_overhang_m: float = 0.30  # 从雷达到车头最前端的距离（米）
    rear_overhang_m: float = 0.15  # 从雷达到车尾最后端的距离（米）
    safety_margin_m: float = 0.10  # 车体包络圆盘外额外增加的安全裕量（米）
    lookahead_m: float = 1.50  # 要检查的圆盘中心沿候选方向前移的距离（米）
    max_distance_m: float = 5.0  # 净空、评分和显示的距离上限（米）
    no_return_distance_m: float = 5.0  # +inf 的有限规划范围（米），再受 range_max 限制；0 禁用
    min_scan_finite_ratio: float = 0.05  # 整帧真实有限回波最低比例，防止全无回波/零散点驱动车辆
    front_half_angle_deg: float = 75.0  # 前向搜索范围的半角（度），如 75 表示左右各 75 度
    bin_angle_deg: float = 1.0  # 目标分箱角宽（度），不是原始雷达相邻束的角度间隔
    min_bin_valid_ratio: float = 0.5  # 单箱可用射线（有限回波或受限 +inf）的最低比例
    min_window_valid_ratio: float = 0.6  # 候选窗口内可用原始射线的最低比例，不要求都有有限回波
    max_interpolate_gap_deg: float = 2.0  # 允许补齐的缺测段最大角宽（度），0 表示不补齐
    interpolate_range_jump_m: float = 0.5  # 补齐时允许的两端真实测距的最大差值（米）
    max_inferred_ratio: float = 0.2  # 候选方向窗口内，补齐分箱占全部分箱的最高比例（0～1）
    forward_weight: float = 0.25  # 前行倾向的评分权重（无量纲），越大越偏向正前方
    continuity_weight: float = 0.10  # 方向变化的惩罚权重（无量纲），越大越倾向延续上一目标

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value):
                raise ValueError(f'{name} 必须是有限数值')
        for name in ('vehicle_width_m', 'lookahead_m', 'max_distance_m',
                     'bin_angle_deg'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} 必须大于 0')
        for name in ('front_overhang_m', 'rear_overhang_m', 'safety_margin_m',
                     'max_interpolate_gap_deg', 'interpolate_range_jump_m',
                     'forward_weight', 'continuity_weight', 'no_return_distance_m'):
            if getattr(self, name) < 0:
                raise ValueError(f'{name} 不能为负')
        for name in ('min_bin_valid_ratio', 'min_window_valid_ratio',
                     'max_inferred_ratio', 'min_scan_finite_ratio'):
            if not 0 <= getattr(self, name) <= 1:
                raise ValueError(f'{name} 必须在 [0, 1] 内')
        if self.min_bin_valid_ratio == 0 or self.min_window_valid_ratio == 0:
            raise ValueError('有效点比例必须大于 0')
        if not 0 < self.front_half_angle_deg < 90:
            raise ValueError('前向搜索半角必须在 (0, 90) 度内')
        if not 0.25 <= self.bin_angle_deg <= 5:
            raise ValueError('角度分箱应在 [0.25, 5] 度内')
        if self.max_interpolate_gap_deg > 5:
            raise ValueError('只允许补齐不超过 5 度的短缺测段')
        if self.max_distance_m <= self.lookahead_m + self.radius_m:
            raise ValueError('max_distance_m 应大于 lookahead_m 加车体包络半径')
        if self.no_return_distance_m > self.max_distance_m:
            raise ValueError('no_return_distance_m 不能大于 max_distance_m')

    @property
    def radius_m(self):
        # 用以雷达为圆心、包住车体四角的圆盘保守近似车体。
        # 这样直线扫掠时也包含前后悬，不会仅考虑车辆宽度而漏掉车角。
        return math.hypot(
            self.vehicle_width_m / 2,
            max(self.front_overhang_m, self.rear_overhang_m),
        ) + self.safety_margin_m


@dataclass
class ScanData:
    """一帧整理后的观测；原始点数组长度为 N，分箱数组长度为 M，两者不能混用。"""

    ranges: np.ndarray  # N 个原始距离（米），保留原下标及 inf、NaN 等特殊值
    angles: np.ndarray  # N 个点对应的车体方向（弧度），含安装偏角并归一化到 [-pi, pi)
    valid: np.ndarray  # N 个布尔标记：True 表示该原始测距为正有限值且处于量程内
    positive_inf: np.ndarray  # N 个布尔标记：原始 +inf；无回波，不是一个真实的远处障碍点
    too_close: np.ndarray  # N 个布尔标记：True 表示 -inf 或正距离低于最小量程，触发停止
    bin_angles: np.ndarray  # M 个角度分箱的中心方向（弧度），按 [-pi, pi) 顺序排列
    bin_ranges: np.ndarray  # M 个箱的最近有限距离/受限无回波范围/短缺口补齐值；不足时为 NaN
    inferred: np.ndarray  # M 个布尔标记：True 表示该箱距离由短缺口补齐，不是真实观测
    bin_step: float  # 实际分箱角宽（弧度）= 2*pi/M，不等同于原始 angle_increment
    supported: np.ndarray  # N 个布尔标记：有效有限回波或启用了有限规划范围的 +inf
    no_return_limit_m: float  # 本帧 +inf 的实际规划上限 min(参数, range_max)；0 表示不采用

    @property
    def valid_ratio(self):
        """整帧原始点的有效比例（0～1），不是某个窗口或分箱的有效比例。"""
        return float(np.mean(self.valid)) if self.valid.size else 0.0


@dataclass
class Gap:
    """前向连续开口的角度边界，以及指定检查距离处的宽度估计。"""

    start: float  # 开口较小一侧的角度边界（弧度），不是原始数组下标
    end: float  # 开口较大一侧的角度边界（弧度），不是最后一束激光的中心角
    width_m: float  # 在 lookahead_m + radius_m 距离处计算的开口弦宽（米）


@dataclass
class Candidate:
    """通过观测及车体检查的一个候选行驶方向。"""

    angle: float  # 目标车体方向（弧度），0 朝前、正值朝左；不是 steer 控制量
    clearance_m: float  # 圆盘直线净空（米），受已测障碍、max_distance_m 和无回波规划范围限制
    valid_ratio: float  # 此候选角度窗口内，原始有效点占全部原始点的比例（0～1）
    mean_distance_m: float  # 窗口内可用射线的有限规划距离均值，包含受限 +inf（米）
    inferred_ratio: float  # 此候选窗口中，补齐分箱占全部分箱的比例（0～1）
    score: float  # 达标净空、规划距离均值、前行倾向和方向连续性评分，越大越优
    support_ratio: float  # 窗口内有限回波与受限 +inf 合计占原始射线的比例（0～1）
    no_return_ratio: float  # 窗口内原始 +inf 占原始射线的比例（0～1），不能当成有限回波比例


@dataclass
class PlanResult:
    """一帧完整分析结果，供控制映射、日志和绘图共同使用。"""

    scan: ScanData  # 处理后的扫描数据，包括原始点、有效性掩码和分箱
    status: str  # 机器可读状态码，如 OK、NO_OBSERVED_GAP、TOO_CLOSE
    reason: str  # 对本次状态的中文解释，便于理解选择方向或停止的原因
    gaps: list = field(default_factory=list)  # 粗略深度/宽度开口，仅供诊断，不再限制候选生成
    candidates: list = field(default_factory=list)  # 通过全部筛选、可以参与评分的 Candidate 列表
    target: Candidate = None  # 最终得分最高的候选；None 表示无可信目标，控制层应停止
    front_clearance_m: float = None  # 正前方规划净空（米），可能含受限无回波假设；None 表示覆盖不足


def prepare_scan(scan, config):
    """保留原始数组和掩码，另建固定角宽分箱，不删除任何原始下标。"""
    ranges = np.asarray(scan.ranges, dtype=float)
    if ranges.ndim != 1:
        raise ValueError('ranges 必须是一维数组')
    if (not math.isfinite(scan.angle_min)
            or not math.isfinite(scan.angle_increment)
            or scan.angle_increment == 0
            or not math.isfinite(scan.range_min)
            or scan.range_min < 0
            or math.isnan(scan.range_max)
            or scan.range_max <= scan.range_min):
        raise ValueError('雷达角度或量程元数据无效')

    # 相邻下标的角度由本帧元数据决定，点数变化不影响物理方向的含义。
    angles = wrap_angle(
        scan.angle_min + np.arange(ranges.size) * scan.angle_increment
        + math.radians(config.sensor_yaw_offset_deg)
    )
    valid = (np.isfinite(ranges) & (ranges > 0)
             & (ranges >= scan.range_min) & (ranges <= scan.range_max))
    positive_inf = np.isposinf(ranges)
    # ROS 测距约定中 -inf 表示太近，不能把它当作可插值的普通缺口。
    too_close = np.isneginf(ranges) | (
        np.isfinite(ranges) & (ranges > 0) & (ranges < scan.range_min))

    # +inf 不再与 NaN/零值混为一类。这里只建立有上限的规划假设，
    # 原始 ranges 和 valid 保持不变，也不会把 +inf 伪造成障碍点。
    no_return_limit = min(config.no_return_distance_m, scan.range_max)
    if no_return_limit <= scan.range_min:
        no_return_limit = 0.0
    supported = valid | (positive_inf & (no_return_limit > 0))
    planning_ranges = np.where(valid, np.minimum(ranges, config.max_distance_m),
                               no_return_limit)

    # 分箱仅用于连续角宽和观测覆盖检查。每箱用最近有效点，避免均值掩盖近障碍。
    count = int(math.ceil(360.0 / config.bin_angle_deg))
    step = 2 * math.pi / count
    bin_angles = -math.pi + (np.arange(count) + 0.5) * step
    # 精确位于分箱边界的角度可能有浮点舍入误差；微小容差避免平白制造空箱。
    indexes = np.floor((angles + math.pi) / step + 1e-9).astype(int) % count
    total = np.bincount(indexes, minlength=count)
    good = np.bincount(indexes[supported], minlength=count)
    fraction = np.divide(good, total, out=np.zeros(count), where=total > 0)
    nearest = np.full(count, np.inf)
    np.minimum.at(nearest, indexes[supported], planning_ranges[supported])
    known = (good > 0) & (fraction >= config.min_bin_valid_ratio)
    blocked = np.bincount(indexes[too_close], minlength=count) > 0  # 存在很近障碍的分箱
    known &= ~blocked  # 不存在很近障碍
    depths = np.where(known, nearest, np.nan)  # 深度可靠的分箱使用最近值，否则使用NaN
    inferred = np.zeros(count, dtype=bool)

    # 插值只处理很短的无效缺口；+inf 自身已有独立的有限规划语义。
    # 两端必须有真实有限回波，不能靠两个无回波端点把 NaN 区域补成可通行。
    # 按圆环处理，因此 359° 与 0° 之间的小缺口也能正确连接。
    anchors = np.flatnonzero(known)
    finite_bins = np.bincount(indexes[valid], minlength=count) > 0
    finite_nearest = np.full(count, np.inf)
    np.minimum.at(finite_nearest, indexes[valid], ranges[valid])
    if anchors.size >= 2:
        max_hole = math.radians(config.max_interpolate_gap_deg)
        for left, right in zip(anchors, np.roll(anchors, -1)):
            missing = (right - left - 1) % count  # 相邻两个深度有效分箱的索引差值，也即中间的空洞宽度
            if (not finite_bins[left] or not finite_bins[right]
                    or missing == 0 or missing * step > max_hole + 1e-9
                    or abs(finite_nearest[left] - finite_nearest[right])
                    > config.interpolate_range_jump_m):
                continue
            hole = (left + np.arange(1, missing + 1)) % count
            if blocked[hole].any():
                continue
            depths[hole] = min(finite_nearest[left], finite_nearest[right],
                               config.max_distance_m)
            inferred[hole] = True

    return ScanData(ranges, angles, valid, positive_inf, too_close,
                    bin_angles, depths, inferred, step, supported, no_return_limit)


def _clearance(angle, points, config):
    """圆盘沿候选直线移动，到首次接触已测障碍之前的距离。"""
    if not points.size:
        return 0.0
    along = points[:, 0] * math.cos(angle) + points[:, 1] * math.sin(angle)
    across = -points[:, 0] * math.sin(angle) + points[:, 1] * math.cos(angle)
    within_width = np.abs(across) < config.radius_m
    half_chord = np.sqrt(np.maximum(config.radius_m ** 2 - across ** 2, 0))
    relevant = within_width & (along + half_chord >= 0)
    contact = along[relevant] - half_chord[relevant]
    return float(np.clip(np.min(contact, initial=config.max_distance_m),
                         0.0, config.max_distance_m))


def _planning_clearance(angle, points, data, config, raw_window):
    """真实点决定碰撞距离；无回波仅限制检查范围，不产生虚构障碍。"""
    clearance = _clearance(angle, points, config)
    if np.any(data.positive_inf & data.supported & raw_window):
        # 需要整个末端圆盘都留在无回波的有限规划范围内，故减去包络半径。
        clearance = min(clearance, max(0.0, data.no_return_limit_m - config.radius_m))
    return clearance


def analyze_scan(scan, config, previous_target=None):
    """返回感知和选方向结果；不发布指令，便于用合成数据和 bag 单独验证。"""
    try:
        data = prepare_scan(scan, config)
    except (ValueError, TypeError, OverflowError) as error:
        empty = np.empty(0)
        data = ScanData(empty, empty, empty.astype(bool), empty.astype(bool), empty.astype(bool),
                        empty, empty, empty.astype(bool), 0.0, empty.astype(bool), 0.0)
        return PlanResult(data, 'INVALID_METADATA', str(error))
    if data.too_close.any():
        return PlanResult(data, 'TOO_CLOSE', '传感器报告近于最小量程，停止并检查近障碍')
    if not data.valid.any():
        return PlanResult(data, 'NO_VALID_RANGES', '本帧没有有效的有限测距')
    if data.valid_ratio < config.min_scan_finite_ratio:
        return PlanResult(data, 'INSUFFICIENT_FINITE_SUPPORT',
                          '整帧真实有限回波过少，不能仅凭大量无回波启动')

    result = PlanResult(data, 'NO_OBSERVED_GAP', '没有观测覆盖足够的前向候选')
    points = np.column_stack((
        data.ranges[data.valid] * np.cos(data.angles[data.valid]),
        data.ranges[data.valid] * np.sin(data.angles[data.valid]),
    ))
    endpoint = config.lookahead_m + config.radius_m
    half_window = math.asin(config.radius_m / endpoint)
    front = np.abs(data.bin_angles) <= math.radians(config.front_half_angle_deg)  # 前向搜索角度范围[-75, 75]
    open_bins = front & np.isfinite(data.bin_ranges) & (data.bin_ranges >= endpoint)

    # 开口只作为粗略深度/宽度诊断，不再作为生成候选的先决条件。
    # +inf 可提供有限规划范围，因此不需要走廊尽头存在连续的有限回波。
    edges = np.diff(np.r_[False, open_bins, False].astype(int))
    for start, end in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
        low = data.bin_angles[start] - data.bin_step / 2
        high = data.bin_angles[end - 1] + data.bin_step / 2
        width = 2 * endpoint * math.sin((high - low) / 2)  # 开口弦宽
        if width >= 2 * config.radius_m:
            result.gaps.append(Gap(low, high, width))

    # “前方净空”也受到无回波规划上限限制；不把 NaN 或未扫描方向当成空旷。
    center_window = np.abs(data.bin_angles) <= half_window
    if center_window.any() and np.all(np.isfinite(data.bin_ranges[center_window])):
        raw_center = np.abs(data.angles) <= half_window
        result.front_clearance_m = _planning_clearance(0.0, points, data, config, raw_center)

    # 先在前向角度空间采样，再逐个检查。即使中心射线全是 +inf，0° 也会接受检查。
    # 不再要求候选窗口内每束射线都在同一个远距离处形成“背景墙”。
    search_half_angle = math.radians(config.front_half_angle_deg)
    heading_mask = np.abs(data.bin_angles) + half_window <= search_half_angle
    headings = data.bin_angles[heading_mask]
    if half_window <= search_half_angle:
        headings = np.sort(np.r_[headings, 0.0])
    has_coverage = False
    for angle in headings:
        raw_window = np.abs(wrap_angle(data.angles - angle)) <= half_window
        support = float(np.mean(data.supported[raw_window])) if raw_window.any() else 0.0
        band = np.abs(wrap_angle(data.bin_angles - angle)) <= half_window
        if not band.any():
            continue
        inferred_ratio = float(np.mean(data.inferred[band]))
        if (support < config.min_window_valid_ratio
                or not np.all(np.isfinite(data.bin_ranges[band]))
                or inferred_ratio > config.max_inferred_ratio):
            continue
        has_coverage = True
        # 所有真实有限点都参加扫掠，低覆盖箱或 inf 邻近的孤立近点也不能漏掉。
        clearance = _planning_clearance(float(angle), points, data, config, raw_window)
        if clearance < config.lookahead_m:
            continue
        usable = raw_window & data.supported
        distances = np.where(data.valid[usable],
                             np.minimum(data.ranges[usable], config.max_distance_m),
                             data.no_return_limit_m)
        mean = float(np.mean(distances))
        finite_ratio = float(np.mean(data.valid[raw_window]))
        no_return_ratio = float(np.mean(data.positive_inf[raw_window]))
        # 本轮先修复候选入口；走廊航向/中线参考和新的巡航评分在下一步引入。
        change = abs(float(wrap_angle(angle - previous_target))) \
            if previous_target is not None else 0.0
        # 净空达到本次前瞻要求后不再额外奖励。否则仅因为一个 inf 的规划
        # 上限稍短，算法也会舍弃正前方去追逐更远的有限回波。
        score = (min(clearance, config.lookahead_m) / config.max_distance_m
                 + 0.15 * mean / config.max_distance_m
                 + config.forward_weight * math.cos(angle)
                 - config.continuity_weight * change / math.pi)
        result.candidates.append(Candidate(
            float(angle), clearance, finite_ratio, mean, inferred_ratio, score,
            support, no_return_ratio))
    if result.candidates:
        result.target = max(result.candidates, key=lambda c: (
            round(c.score, 9), -abs(c.angle), c.angle))
        result.status = 'OK'
        result.reason = '有限回波/受限无回波覆盖及直线车体扫掠检查通过'
    elif has_coverage:
        result.status = 'NO_CLEAR_PATH'
        result.reason = '存在覆盖足够的方向，但真实障碍或规划范围不允许所需车体净空'
    return result

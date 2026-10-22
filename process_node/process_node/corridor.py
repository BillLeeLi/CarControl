"""直走廊巡航参考：真实墙线、里程计运动补偿和有期限的降级保持。"""

from dataclasses import dataclass
import math

import numpy as np

from process_node.perception import wrap_angle


@dataclass(frozen=True)
class CorridorConfig:
    """节点将字段注册为 corridor_ 开头的参数；几何值和增益需实车核验。"""

    sensor_x_m: float = 0.0  # 雷达原点相对 odom 子坐标系原点的前向偏移（米）
    sensor_y_m: float = 0.0  # 雷达原点相对 odom 子坐标系原点的左向偏移（米）
    fit_range_m: float = 4.5  # 拟合使用的最大真实测距（米），不采用 inf 或插值点
    fit_front_m: float = 4.0  # 拟合区域的车体前向边界（米）
    fit_rear_m: float = 0.3  # 拟合区域向车后保留的距离（米）
    voxel_m: float = 0.05  # 空间网格采样尺寸（米），避免近处密集点支配墙线
    ransac_trials: int = 80  # 每次找墙线的随机采样次数，使用固定种子保证可重复
    max_lines: int = 6  # 最多提取的候选墙线数，允许墙面被门洞分开
    line_tolerance_m: float = 0.06  # 墙线内点的最大垂直误差（米）
    min_inliers: int = 12  # 网格采样后，一条墙线至少需要的真实支持点数
    min_span_m: float = 1.2  # 支持点沿墙方向至少覆盖的长度（米）
    max_rmse_m: float = 0.04  # 墙线内点允许的均方根误差（米）
    min_wall_distance_m: float = 0.25  # 墙线离车体原点的最小距离（米），仅用于拟合
    max_heading_deg: float = 45.0  # 本轮可识别的墙线相对车头方向范围（度）
    parallel_tolerance_deg: float = 8.0  # 左右墙走向允许的最大夹角（度）
    min_width_m: float = 0.9  # 可接受的走廊宽度下限（米），不是预先写死路线宽度
    max_width_m: float = 3.5  # 可接受的走廊宽度上限（米）
    max_heading_change_deg: float = 12.0  # 运动补偿后相邻观测的最大走向变化（度）
    max_center_change_m: float = 0.20  # 运动补偿后中线允许的观测跳变（米）
    max_width_change_m: float = 0.25  # 已跟踪宽度允许的观测跳变（米）
    confirm_frames: int = 3  # 初始化时连续多少帧一致的双墙观测才能进入巡航
    filter_tau_s: float = 0.25  # 走向、中线和宽度的消息时间低通时间常数（秒）
    max_update_gap_s: float = 0.5  # 超过此扫描间隔重新确认走廊（秒）
    single_wall_timeout_s: float = 0.6  # 距最近双墙确认最多可单墙推断多久（秒）
    hold_timeout_s: float = 0.3  # 两侧均缺失时最多保持多久（秒）
    max_unobserved_travel_m: float = 0.2  # 距最近墙面观测最多允许移动的距离（米）
    goal_distance_m: float = 1.5  # 沿走廊方向放置目标点的距离（米）
    lateral_gain: float = 1.0  # 横向回中修正系数（无量纲）
    max_lateral_correction_deg: float = 20.0  # 横向偏移产生的修正角上限（度）
    max_target_deg: float = 45.0  # 巡航总目标角上限（度），超出时停止，留给后续转弯行为
    candidate_tolerance_deg: float = 5.0  # 允许候选偏离巡航参考的最大角度（度）
    recovery_heading_deg: float = 15.0  # 航向误差超过此角度时降低目标速度（度）
    recovery_offset_m: float = 0.25  # 横向偏移超过此距离时降低目标速度（米）
    slow_speed_scale: float = 0.5  # 单墙、历史保持或较大偏差时的请求速度比例

    def __post_init__(self):
        integers = ('ransac_trials', 'max_lines', 'min_inliers', 'confirm_frames')
        nonnegative = ('fit_rear_m', 'filter_tau_s', 'lateral_gain')
        for name, value in vars(self).items():
            if not math.isfinite(value):
                raise ValueError(f'corridor_{name} 必须是有限值')
            if name in ('sensor_x_m', 'sensor_y_m'):
                continue
            if name in integers and (type(value) is not int or value < 1):
                raise ValueError(f'corridor_{name} 必须是正整数')
            if value < 0 or (name not in nonnegative and value == 0):
                raise ValueError(f'corridor_{name} 超出范围')
        if (self.min_width_m >= self.max_width_m or self.max_lines < 2
                or self.min_inliers < 3 or self.fit_front_m > self.fit_range_m
                or self.max_rmse_m > self.line_tolerance_m):
            raise ValueError('走廊拟合的宽度、点数、距离或误差配置不合理')
        for name in ('max_heading_deg', 'parallel_tolerance_deg',
                     'max_heading_change_deg', 'max_lateral_correction_deg',
                     'max_target_deg', 'candidate_tolerance_deg', 'recovery_heading_deg'):
            if getattr(self, name) >= 90:
                raise ValueError(f'corridor_{name} 必须小于 90 度')
        if self.slow_speed_scale > 1 or self.hold_timeout_s > self.single_wall_timeout_s:
            raise ValueError('走廊降速比例或历史保持时间不合理')


@dataclass(frozen=True)
class WallLine:
    """车体坐标中的真实点拟合结果，法向量始终指向墙线切向的左侧。"""

    heading_rad: float  # 墙线朝前的切向角，弧度
    offset_m: float  # 墙线方程 n·p=offset，左墙为正、右墙为负（米）
    center_xy: tuple  # 支持点质心，车体坐标（米）
    inliers: int  # 网格采样后支持该线的真实点数
    span_m: float  # 支持点沿墙方向的跨度（米）
    rmse_m: float  # 内点到墙线的均方根距离（米）


@dataclass(frozen=True)
class CorridorEstimate:
    """感知参考与可用状态一起传递；可信度为诊断分数，不是概率。"""

    status: str  # TRACKING、SINGLE_WALL、HOLD 或等待/丢失等状态，带 CORRIDOR_ 前缀
    reason: str  # 本次采用或拒绝观测的原因
    heading_rad: float = None  # 走廊走向相对当前车头的角度（弧度）
    lateral_error_m: float = None  # 中线在车体原点左侧的距离（米）；正值表示应向左回中
    width_m: float = None  # 当前估计的左右墙间距（米）
    target_angle_rad: float = None  # 回正和回中共同生成的参考角度（弧度）
    target_point_xy: tuple = None  # 车体坐标中的参考目标点（米）
    confidence: float = 0.0  # 墙线质量与保持时间综合诊断分数，范围 0～1
    measurement_age_s: float = None  # 距最近采用的墙面观测的消息时间（秒）
    pair_age_s: float = None  # 距最近双墙确认的消息时间（秒）
    speed_scale: float = 0.0  # 本参考允许的请求速度比例，失效时为 0
    observed_walls: tuple = ()  # 本帧拟合出的真实候选墙线，仅用于诊断
    sensor_offset_xy: tuple = (0.0, 0.0)  # 雷达原点在车体坐标中的偏移（米），供绘图变换

    @property
    def usable(self):
        return self.target_angle_rad is not None and self.status in (
            'CORRIDOR_TRACKING', 'CORRIDOR_SINGLE_WALL', 'CORRIDOR_HOLD')


def _normal(heading):
    return np.array([-math.sin(heading), math.cos(heading)])


def _wall_points(data, config):
    # 只在拟合中采样；原始扫描中的所有近点仍由 perception 做碰撞检查。
    good = data.valid & (data.ranges <= config.fit_range_m)
    p = np.column_stack((data.ranges[good] * np.cos(data.angles[good]),
                         data.ranges[good] * np.sin(data.angles[good])))
    p += [config.sensor_x_m, config.sensor_y_m]
    p = p[(p[:, 0] >= -config.fit_rear_m) & (p[:, 0] <= config.fit_front_m)]
    if not len(p):
        return p
    cells = np.floor(p / config.voxel_m).astype(np.int64)
    _, inverse = np.unique(cells, axis=0, return_inverse=True)
    count = np.bincount(inverse)
    return np.column_stack([np.bincount(inverse, weights=p[:, axis]) / count
                            for axis in (0, 1)])


def fit_walls(data, config):
    """固定种子的 RANSAC + 正交最小二乘；不依赖 Scipy 或额外第三方库。"""
    remaining = _wall_points(data, config)
    rng = np.random.default_rng(0)
    walls = []
    for _ in range(config.max_lines):
        if len(remaining) < config.min_inliers:
            break
        best, best_key = None, None
        for _ in range(config.ransac_trials):
            a, b = remaining[rng.choice(len(remaining), 2, replace=False)]
            delta = b - a
            if np.linalg.norm(delta) < config.min_span_m / 2:
                continue
            if delta[0] < 0:
                delta = -delta
            heading = math.atan2(delta[1], delta[0])
            if abs(heading) > math.radians(config.max_heading_deg):
                continue
            normal = _normal(heading)
            offset = float(a @ normal)
            if abs(offset) < config.min_wall_distance_m:
                continue
            residual = np.abs(remaining @ normal - offset)
            inside = residual <= config.line_tolerance_m
            if inside.sum() < config.min_inliers:
                continue
            projection = remaining[inside] @ np.array([math.cos(heading), math.sin(heading)])
            span = float(np.ptp(projection))
            if span < config.min_span_m:
                continue
            key = (int(inside.sum()), span, -float(np.mean(residual[inside] ** 2)))
            if best_key is None or key > best_key:
                best, best_key = inside, key
        if best is None:
            break
        # 用内点质心和主方向细化，避免 y=kx+b 在倾斜时产生方向偏置。
        inside = best
        for _ in range(2):
            points = remaining[inside]
            center = points.mean(axis=0)
            _, vectors = np.linalg.eigh((points - center).T @ (points - center))
            tangent = vectors[:, -1]
            if tangent[0] < 0:
                tangent = -tangent
            heading = math.atan2(tangent[1], tangent[0])
            normal = _normal(heading)
            offset = float(center @ normal)
            residual = np.abs(remaining @ normal - offset)
            inside = residual <= config.line_tolerance_m
            if inside.sum() < config.min_inliers:
                break
        points = remaining[inside]
        if len(points) < config.min_inliers:
            break
        span = float(np.ptp(points @ tangent))
        rmse = float(np.sqrt(np.mean(residual[inside] ** 2)))
        remaining = remaining[~inside]
        if (abs(heading) <= math.radians(config.max_heading_deg)
                and abs(offset) >= config.min_wall_distance_m
                and span >= config.min_span_m and rmse <= config.max_rmse_m):
            walls.append(WallLine(heading, offset, tuple(center), len(points), span, rmse))
    return tuple(walls)


class CorridorTracker:
    """把中线存为 odom 坐标中的直线；每帧补偿车体平移和转动后再融合。"""

    def __init__(self, config=CorridorConfig()):
        self.config = config
        self.reset()

    def reset(self):
        self._model = None  # (odom 中的走向, 中线法向偏移, 宽度)
        self._pending = None
        self._confirmations = 0
        self._last_scan_ns = None
        self._last_observation_ns = None
        self._last_pair_ns = None
        self._last_observation_xy = None
        self._confidence = 0.0

    @staticmethod
    def _to_body(model, pose):
        heading, center, width = model
        x, y, yaw = pose
        return (float(wrap_angle(heading - yaw)),
                center - float(_normal(heading) @ [x, y]), width)

    @staticmethod
    def _to_world(measurement, pose):
        heading, center, width = measurement
        heading = float(wrap_angle(heading + pose[2]))
        return heading, center + float(_normal(heading) @ pose[:2]), width

    def _consistent(self, measured, predicted):
        cfg = self.config
        return (abs(float(wrap_angle(measured[0] - predicted[0])))
                <= math.radians(cfg.max_heading_change_deg)
                and abs(measured[1] - predicted[1]) <= cfg.max_center_change_m
                and abs(measured[2] - predicted[2]) <= cfg.max_width_change_m)

    def _pair(self, walls, prediction):
        cfg = self.config
        choices = []
        for left in walls:
            if left.offset_m <= 0:
                continue
            for right in walls:
                if right.offset_m >= 0:
                    continue
                difference = abs(float(wrap_angle(left.heading_rad - right.heading_rad)))
                if difference > math.radians(cfg.parallel_tolerance_deg):
                    continue
                heading = (left.heading_rad + right.heading_rad) / 2
                normal = _normal(heading)
                l, r = float(normal @ left.center_xy), float(normal @ right.center_xy)
                width = l - r
                if not (l > 0 > r and cfg.min_width_m <= width <= cfg.max_width_m):
                    continue
                measured = heading, (l + r) / 2, width
                if prediction is not None and not self._consistent(measured, prediction):
                    continue
                score = left.span_m + right.span_m + 0.02 * (left.inliers + right.inliers)
                score -= 5 * difference + 10 * (left.rmse_m + right.rmse_m)
                if prediction is not None:
                    score -= 2 * (abs(measured[1] - prediction[1]) / cfg.max_center_change_m
                                  + abs(width - prediction[2]) / cfg.max_width_change_m)
                quality = min(1.0, min(left.span_m, right.span_m) / (2 * cfg.min_span_m))
                quality *= max(0.0, 1 - max(left.rmse_m, right.rmse_m) / cfg.line_tolerance_m)
                choices.append((score, measured, quality))
        return max(choices, key=lambda item: item[0]) if choices else None

    def _accept(self, measured, pose, stamp_ns, confidence, pair):
        if self._model is not None:
            previous = self._to_body(self._model, pose)
            dt = (stamp_ns - self._last_observation_ns) * 1e-9
            alpha = 1.0 if self.config.filter_tau_s == 0 else \
                -math.expm1(-dt / self.config.filter_tau_s)
            measured = (previous[0] + alpha * float(wrap_angle(measured[0] - previous[0])),
                        previous[1] + alpha * (measured[1] - previous[1]),
                        previous[2] + alpha * (measured[2] - previous[2]))
        # 先在当前车体原点附近融合再存到世界，避免 odom 大坐标放大角度滤波误差。
        self._model = self._to_world(measured, pose)
        self._last_observation_ns = stamp_ns
        self._last_observation_xy = np.array(pose[:2])
        self._confidence = confidence
        if pair:
            self._last_pair_ns = stamp_ns

    def _estimate(self, status, reason, pose, stamp_ns, walls):
        if self._model is None:
            return CorridorEstimate(status, reason, observed_walls=walls)
        cfg = self.config
        heading, error, width = self._to_body(self._model, pose)
        age = (stamp_ns - self._last_observation_ns) * 1e-9
        pair_age = (stamp_ns - self._last_pair_ns) * 1e-9
        correction = float(np.clip(math.atan2(cfg.lateral_gain * error, cfg.goal_distance_m),
                                   -math.radians(cfg.max_lateral_correction_deg),
                                   math.radians(cfg.max_lateral_correction_deg)))
        target = float(wrap_angle(heading + correction))
        normal = _normal(heading)
        tangent = np.array([math.cos(heading), math.sin(heading)])
        goal = cfg.goal_distance_m * tangent + cfg.goal_distance_m * math.tan(correction) * normal
        degraded = status != 'CORRIDOR_TRACKING'
        recovery = (abs(heading) > math.radians(cfg.recovery_heading_deg)
                    or abs(error) > cfg.recovery_offset_m)
        scale = cfg.slow_speed_scale if degraded or recovery else 1.0
        confidence = self._confidence * math.exp(-age / cfg.hold_timeout_s)
        if abs(target) > math.radians(cfg.max_target_deg):
            status, reason, target, scale = ('CORRIDOR_OUT_OF_RANGE',
                                            '纠偏角超出直走廊巡航范围，等待后续行为处理', None, 0.0)
        return CorridorEstimate(status, reason, heading, error, width, target, tuple(goal),
                                confidence, age, pair_age, scale, walls,
                                (cfg.sensor_x_m, cfg.sensor_y_m))

    def update(self, data, feedback, stamp_ns):
        """反馈必须已按本帧激光关联；全部计时使用激光消息时间。"""
        if type(stamp_ns) is not int or stamp_ns < 0:
            self.reset()
            return CorridorEstimate('CORRIDOR_TIME_INVALID', '激光时间戳不合法')
        if not feedback.valid:
            self.reset()
            return CorridorEstimate('CORRIDOR_NO_FEEDBACK', feedback.reason)
        state = feedback.state
        pose = (state.x_m, state.y_m, state.yaw_rad)
        if not all(math.isfinite(v) for v in pose):
            self.reset()
            return CorridorEstimate('CORRIDOR_NO_FEEDBACK', '运动位姿包含非有限值')
        if self._last_scan_ns is not None and stamp_ns <= self._last_scan_ns:
            self.reset()
            return CorridorEstimate('CORRIDOR_TIME_INVALID', '激光时间重复或回跳，重新确认走廊')
        if (self._last_scan_ns is not None and (stamp_ns - self._last_scan_ns) * 1e-9
                > self.config.max_update_gap_s):
            self.reset()
        self._last_scan_ns = stamp_ns
        cfg = self.config
        # 历史一旦过期就不能被一帧新观测复活，必须重新通过连续双墙确认。
        if self._model is not None:
            age = (stamp_ns - self._last_observation_ns) * 1e-9
            pair_age = (stamp_ns - self._last_pair_ns) * 1e-9
            travel = np.linalg.norm(np.array(pose[:2]) - self._last_observation_xy)
            if (age > cfg.hold_timeout_s or pair_age > cfg.single_wall_timeout_s
                    or travel > cfg.max_unobserved_travel_m):
                self._model = self._pending = None
                self._confirmations = 0
        prediction = self._to_body(self._model, pose) if self._model is not None else None
        walls = fit_walls(data, cfg)
        pair = self._pair(walls, prediction)
        if pair is not None:
            _, measured, quality = pair
            if self._model is None:
                pending = self._to_body(self._pending, pose) if self._pending is not None else None
                self._confirmations = self._confirmations + 1 \
                    if pending is not None and self._consistent(measured, pending) else 1
                self._pending = self._to_world(measured, pose)
                if self._confirmations < cfg.confirm_frames:
                    return CorridorEstimate('CORRIDOR_ACQUIRING',
                                            f'一致双墙确认 {self._confirmations}/{cfg.confirm_frames}',
                                            observed_walls=walls)
            self._accept(measured, pose, stamp_ns, quality, pair=True)
            return self._estimate('CORRIDOR_TRACKING', '双墙走向、宽度及历史一致性通过',
                                  pose, stamp_ns, walls)
        if self._model is not None:
            choices = []
            for wall in walls:
                # 单墙只能利用近期双墙建立的宽度，不凭一个任意墙面初始化中线。
                side = 1 if wall.offset_m > 0 else -1
                measured = (wall.heading_rad,
                            wall.offset_m - side * prediction[2] / 2, prediction[2])
                if self._consistent(measured, prediction):
                    choices.append((wall.span_m - abs(measured[1] - prediction[1]), measured))
            if choices:
                _, measured = max(choices, key=lambda item: item[0])
                self._accept(measured, pose, stamp_ns, self._confidence * 0.8, pair=False)
                return self._estimate('CORRIDOR_SINGLE_WALL', '单墙结合近期双墙宽度，限时降速',
                                      pose, stamp_ns, walls)
            return self._estimate('CORRIDOR_HOLD', '没有一致墙面观测，短时运动补偿并降速',
                                  pose, stamp_ns, walls)
        self._pending = None
        self._confirmations = 0
        return CorridorEstimate('CORRIDOR_LOST', '缺少可信双墙参考，停止并等待重新确认',
                                observed_walls=walls)


def select_cruise_candidate(candidates, estimate, config):
    """仅在巡航参考附近选择已通过感知检查的方向，不追逐侧面空旷区域。"""
    if not estimate.usable:
        return None
    close = [c for c in candidates if abs(float(wrap_angle(c.angle - estimate.target_angle_rad)))
             <= math.radians(config.candidate_tolerance_deg)]
    return min(close, key=lambda c: (abs(float(wrap_angle(c.angle - estimate.target_angle_rad))),
                                     -c.score, abs(c.angle))) if close else None

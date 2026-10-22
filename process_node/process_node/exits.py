"""道路出口的几何假设与 odom 跟踪；本模块不生成任何控制指令。"""

from dataclasses import dataclass
import math

import numpy as np

from process_node.perception import wrap_angle


@dataclass(frozen=True)
class ExitConfig:
    """ROS 参数使用 exit_ 前缀；宽度/深度不是实际转弯可行性保证。"""

    enabled: bool = True  # 是否运行出口诊断；关闭不改变巡航控制
    search_front_m: float = 4.0  # 沿接近走廊向前搜索侧向入口的距离（米）
    search_rear_m: float = 0.3  # 允许入口中心略在车后，保持经过入口时的关联（米）
    sample_step_m: float = 0.10  # 入口横向与纵深采样间隔（米）
    min_width_m: float = 1.0  # 入口可见宽度下限（米），帮助排除窄门洞
    max_width_m: float = 3.5  # 超宽开阔区域不作为已确认道路出口（米）
    probe_depth_m: float = 1.0  # 从入口向出口方向检查的可见纵深（米）
    min_depth_visible_ratio: float = 0.15  # 入口至少多少比例能观察到完整纵深；其余可能被墙角遮挡
    forward_gate_m: float = 1.3  # 前向延续通道的诊断截面距离（米）
    clearance_m: float = 0.08  # 采样点到观测障碍的额外距离（米）
    direction_offset_deg: float = 20.0  # 侧向通道除正交外还尝试 ±此角度（度）
    wall_band_m: float = 0.25  # 真实结构点接近入口边缘的带宽（米）
    min_wall_points: int = 5  # 支持出口边界的真实点数下限（不是 inf 点）
    min_wall_span_m: float = 0.45  # 出口边界真实点沿出口方向的跨度下限（米）
    confirm_frames: int = 3  # 连续几帧有真实结构支持后确认
    confirm_time_s: float = 0.15  # 确认还须覆盖的最短消息时间（秒）
    match_distance_m: float = 0.8  # odom 坐标中同一侧向入口中心的关联距离（米）
    match_angle_deg: float = 30.0  # 同一出口方向的关联容差（度）
    hold_time_s: float = 0.5  # 暂时不可见时保留编号的最长时间（秒）
    hold_travel_m: float = 0.5  # 暂时不可见时允许的最大车辆位移（米）
    context_timeout_s: float = 3.0  # 失去双墙后沿用接近走廊坐标的最长时间（秒）
    context_travel_m: float = 2.0  # 失去双墙后沿用接近走廊坐标的最大位移（米）
    max_update_gap_s: float = 0.5  # 扫描间隔过长则重新建立跟踪（秒）
    prefer_right: bool = True  # 顺时针路线的右侧分支诊断优先级，不下发转向

    def __post_init__(self):
        for name, value in vars(self).items():
            if name in ('enabled', 'prefer_right'):
                if type(value) is not bool:
                    raise ValueError(f'exit_{name} 必须为布尔值')
                continue
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'exit_{name} 必须为正有限值')
        for name in ('confirm_frames', 'min_wall_points'):
            if type(getattr(self, name)) is not int:
                raise ValueError(f'exit_{name} 必须为整数')
        if (self.min_width_m >= self.max_width_m or self.sample_step_m > self.min_width_m / 3
                or self.search_front_m <= self.forward_gate_m
                or self.probe_depth_m < self.clearance_m + self.sample_step_m
                or not 0 < self.direction_offset_deg < 45
                or self.min_depth_visible_ratio > 1
                or self.match_angle_deg >= 90):
            raise ValueError('exit_ 几何采样、宽度或角度参数不合理')


@dataclass(frozen=True)
class ExitObservation:
    """一帧内可见的入口；坐标在车体原点定义，雷达安装平移已计入。"""

    center_xy: tuple  # 入口截面中心（米）；不是车辆应立即朝向的目标点
    heading_rad: float  # 从入口通向道路内部的方向（弧度），正值向左
    width_m: float  # 当前可见入口宽度（米），遮挡时可能低估
    depth_m: float  # 已检查的出口纵深（米），不表示整条道路长度
    role: str  # FORWARD 为前向延续，SIDE 为接近走廊侧向入口
    structural: bool  # 真实边界点数量和跨度是否达标；无回波本身不能提供此证据
    wall_points: int  # 用于支持入口边界的真实有限点数
    confidence: float  # 几何/结构诊断分数，不是概率或碰撞安全保证
    clipped: bool = False  # 可见入口触及搜索边界，宽度尚未完整观测
    depth_visible_ratio: float = 1.0  # 可见入口中能观测到完整纵深的比例；不等于可通行宽度


@dataclass(frozen=True)
class TrackedExit:
    """返回给绘图和日志的不可变跟踪快照。"""

    track_id: int  # 同一个 odom 入口持续使用的编号，重置后也不复用旧编号
    observation: ExitObservation  # 经过运动补偿的当前车体坐标入口
    state: str  # UNVERIFIED / TENTATIVE / CONFIRMED / HELD
    side: str  # 相对当前接近走廊的 LEFT / FORWARD / RIGHT / BEHIND
    age_s: float  # 距最近实际观测的消息时间（秒）；HELD 不参与推荐
    hits: int  # 连续具有真实结构支持的观测次数


@dataclass(frozen=True)
class ExitResult:
    status: str  # DISABLED / EXIT_NO_CONTEXT / EXIT_TRACKING 等诊断状态
    reason: str  # 当前可用性或拒绝原因的中文说明
    tracks: tuple = ()  # 含暂定及短时保留入口的 TrackedExit 快照
    preferred_id: int = None  # 当前帧确认且可见的建议入口编号，仅供诊断
    recommendation: str = 'NONE'  # RIGHT_AVAILABLE / FORWARD_AVAILABLE / LEFT_AVAILABLE / NONE
    axis_rad: float = None  # 接近走廊在当前车体坐标中的方向（弧度）
    sensor_offset_xy: tuple = (0., 0.)  # 雷达相对车体原点的安装平移（米），供绘图对齐


@dataclass
class _Track:
    track_id: int  # 内部编号
    center_world: np.ndarray  # odom 坐标入口中心（米）
    heading_world: float  # odom 坐标出口方向（弧度）
    observation: ExitObservation  # 最近的几何/证据属性
    first_ns: int  # 当前连续确认序列起点（纳秒）
    last_ns: int  # 最近观测时间（纳秒）
    last_pose: np.ndarray  # 最近看到入口时的车辆位置（米）
    hits: int = 0  # 连续结构支持次数
    confirmed: bool = False  # 是否已完成时间与帧数确认


def _rotation(yaw):
    return np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])


def _known_free(data, body_points, sensor, margin):
    """只查询扫描可见自由区域；未知、插值和墙后区域都不能当作自由。"""
    points = np.asarray(body_points) - sensor
    distance = np.linalg.norm(points, axis=-1)
    angles = np.arctan2(points[..., 1], points[..., 0])
    indexes = np.floor((angles + math.pi) / data.bin_step).astype(int) % len(data.bin_angles)
    # 三个相邻角箱取最小值，减少栅格采样在墙角两侧穿漏；不跨越未知箱。
    nearby = (indexes[..., None] + np.array([-1, 0, 1])) % len(data.bin_angles)
    depths = data.bin_ranges[nearby]
    known = np.isfinite(depths).all(axis=-1) & ~data.inferred[nearby].any(axis=-1)
    return known & (distance + margin < np.min(depths, axis=-1))


def _runs(mask):
    edges = np.diff(np.r_[False, mask, False].astype(int))
    return zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1))


def detect_exits(data, config, axis_rad, center_left_m, width_m, sensor_offset=(0., 0.)):
    """在接近走廊的前/两侧截面检查入口；不要求前方封闭，不检查车辆弯曲轨迹。"""
    cfg = config
    sensor = np.asarray(sensor_offset)
    rotation = _rotation(axis_rad)
    points = np.column_stack((data.ranges[data.valid] * np.cos(data.angles[data.valid]),
                              data.ranges[data.valid] * np.sin(data.angles[data.valid]))) + sensor
    points = points @ rotation  # 从车体坐标转为接近走廊的纵向 u / 横向 v
    observations = []
    depths = np.linspace(cfg.clearance_m, cfg.probe_depth_m,
                         math.ceil((cfg.probe_depth_m-cfg.clearance_m)/cfg.sample_step_m)+1)
    longitudinal = np.arange(-cfg.search_rear_m, cfg.search_front_m + .001, cfg.sample_step_m)
    for side in (-1, 1):
        boundary = center_left_m + side * width_m / 2
        choices = []
        for offset in (0., -cfg.direction_offset_deg, cfg.direction_offset_deg):
            angle = side * math.pi / 2 + math.radians(offset)
            # 沿候选出口方向推进，允许斜墙/三角形区域中的非正交出口。
            u = longitudinal[:, None] + depths * math.cos(angle)
            v = np.broadcast_to(boundary + depths * math.sin(angle), u.shape)
            grid = np.stack((u, v), axis=-1)
            free = _known_free(data, grid @ rotation.T, sensor, cfg.clearance_m)
            # 侧向走廊尚在前方时，墙角会遮住一部分深处。入口宽度用近截面，
            # 深度用确实可见的子区域证明；不能据此声称整条入口宽度都已检查安全。
            clear = free[:, :2].all(axis=1)
            for start, end in _runs(clear):
                visible_width = (end - start) * cfg.sample_step_m
                if visible_width + 1e-9 < cfg.min_width_m:
                    continue
                depth_ratio = float(np.mean(free[start:end].all(axis=1)))
                if depth_ratio < cfg.min_depth_visible_ratio:
                    continue
                lo = longitudinal[start] - cfg.sample_step_m / 2
                hi = longitudinal[end - 1] + cfg.sample_step_m / 2
                # 射线栅格只是可见性检查；再用所有原始真实点拒绝入口区域内的障碍。
                outward = side * (points[:, 1] - boundary)
                projected_u = points[:, 0] - outward / math.sin(abs(angle)) * math.cos(angle)
                inside = ((projected_u > lo) & (projected_u < hi)
                          & (outward > 0) & (outward < cfg.probe_depth_m * abs(math.sin(angle))))
                if inside.any():
                    continue
                # 支持出口纵向边界的真实点。只有很远的 inf 开阔区不会生成结构证据。
                edge = (np.minimum(np.abs(projected_u - lo), np.abs(projected_u - hi))
                        < cfg.wall_band_m)
                wall_mask = edge & (outward > -.1) & (outward < cfg.probe_depth_m + .5)
                wall_points = points[wall_mask]
                span = np.ptp(side * (wall_points[:, 1] - boundary)) if len(wall_points) else 0.
                clipped = start == 0 or end == len(longitudinal)
                structural = (len(wall_points) >= cfg.min_wall_points
                              and span >= cfg.min_wall_span_m
                              and visible_width <= cfg.max_width_m and not clipped)
                observation = ExitObservation(
                    tuple(np.array([(lo + hi) / 2, boundary]) @ rotation.T),
                    float(wrap_angle(axis_rad + angle)), visible_width, cfg.probe_depth_m,
                    'SIDE', structural, len(wall_points), .6+.2*depth_ratio if structural else .3,
                    clipped, depth_ratio)
                choices.append(observation)
        # 相同入口在多个斜向探测中可能重复。优先保留结构证据及接近正交的解释。
        for observation in sorted(choices, key=lambda o: (not o.structural,
                                  abs(float(wrap_angle(o.heading_rad-axis_rad-side*math.pi/2))))):
            if any(np.linalg.norm(np.array(observation.center_xy) - np.array(old.center_xy))
                   < (observation.width_m + old.width_m) / 2
                   for old in observations if old.role == 'SIDE'
                   and side * (np.array(old.center_xy) @ rotation)[1] > side * center_left_m):
                continue
            observations.append(observation)
    # 若看到侧向路口，前向延续必须检查路口另一侧；不能把路口前的短直路当作直行出口。
    mouths = [(np.array(o.center_xy) @ rotation)[0] + o.width_m/2 + cfg.wall_band_m
              for o in observations if o.role == 'SIDE' and not o.clipped]
    gate = max(cfg.forward_gate_m, min(mouths)) if mouths else cfg.forward_gate_m
    lateral = np.arange(-cfg.min_width_m/2, cfg.min_width_m/2 + .001, cfg.sample_step_m)
    forward = np.stack(np.meshgrid(gate + depths, center_left_m + lateral,
                                  indexing='ij'), axis=-1)
    free = _known_free(data, forward @ rotation.T, sensor, cfg.clearance_m).all()
    raw_inside = ((points[:, 0] > gate) & (points[:, 0] < gate + cfg.probe_depth_m)
                  & (np.abs(points[:, 1]-center_left_m) < cfg.min_width_m/2))
    if free and not raw_inside.any():
        wall_mask = ((points[:, 0] > gate-.5) & (points[:, 0] < gate + cfg.probe_depth_m)
                     & (np.abs(np.abs(points[:, 1]-center_left_m)-width_m/2) < cfg.wall_band_m))
        wall_points = points[wall_mask]
        structural = (len(wall_points) >= cfg.min_wall_points
                      and np.ptp(wall_points[:, 0]) >= cfg.min_wall_span_m)
        observations.append(ExitObservation(
            tuple(np.array([gate, center_left_m]) @ rotation.T), axis_rad,
            cfg.min_width_m, cfg.probe_depth_m, 'FORWARD', structural,
            len(wall_points), .8 if structural else .3))
    return observations


class ExitTracker:
    """入口保存在 odom 坐标；短时丢失只保留编号，不把历史入口推荐给控制。"""

    def __init__(self, config=ExitConfig(), sensor_offset=(0., 0.), min_finite_ratio=.05):
        self.config = config
        self.sensor_offset = tuple(sensor_offset)
        self.min_finite_ratio = min_finite_ratio
        self._next_id = 1
        self.reset()

    def reset(self):
        self._tracks = []
        self._last_ns = None
        self._previous_ns = None
        self._context = None  # 世界航向、中线偏移、宽度、更新时间、当时车辆位置

    def update(self, data, feedback, stamp_ns, corridor=None):
        cfg = self.config
        if not cfg.enabled:
            self.reset()
            return ExitResult('DISABLED', '出口诊断已关闭')
        if (not feedback.valid or data.too_close.any() or not data.valid.any()
                or data.valid_ratio < self.min_finite_ratio):
            self.reset()
            return ExitResult('EXIT_INPUT_INVALID', '需要有效扫描和按扫描时间关联的独立反馈')
        state = feedback.state
        pose = np.array([state.x_m, state.y_m])
        yaw = state.yaw_rad
        if (type(stamp_ns) is not int or stamp_ns < 0 or not np.isfinite(pose).all()
                or not math.isfinite(yaw)):
            self.reset()
            return ExitResult('EXIT_INPUT_INVALID', '时间或位姿不合法')
        if self._last_ns is not None and stamp_ns <= self._last_ns:
            self.reset()
            return ExitResult('EXIT_TIME_INVALID', '扫描时间重复或回跳，清除出口历史')
        if self._last_ns is not None and (stamp_ns - self._last_ns)*1e-9 > cfg.max_update_gap_s:
            self.reset()
        self._last_ns = stamp_ns
        if corridor is not None and corridor.status == 'CORRIDOR_TRACKING' and corridor.usable:
            heading = yaw + corridor.heading_rad
            normal = np.array([-math.sin(heading), math.cos(heading)])
            self._context = (heading, corridor.lateral_error_m + normal @ pose,
                             corridor.width_m, stamp_ns, pose.copy())
        if self._context is None:
            return ExitResult('EXIT_NO_CONTEXT', '等待可信双墙建立接近走廊坐标')
        heading, offset, width, last_ns, last_pose = self._context
        if ((stamp_ns - last_ns)*1e-9 > cfg.context_timeout_s
                or np.linalg.norm(pose - last_pose) > cfg.context_travel_m):
            self._context = None
            self._tracks = []
            return ExitResult('EXIT_NO_CONTEXT', '接近走廊历史过期，等待新的双墙参考')
        axis = float(wrap_angle(heading - yaw))
        center = offset - np.array([-math.sin(heading), math.cos(heading)]) @ pose
        observations = detect_exits(data, cfg, axis, center, width, self.sensor_offset)
        rotation = _rotation(yaw)
        # 时间/位移共同限制暂时不可见的入口；每帧先过期，旧确认不能无限复活。
        self._tracks = [t for t in self._tracks if (stamp_ns-t.last_ns)*1e-9 <= cfg.hold_time_s
                        and np.linalg.norm(pose-t.last_pose) <= cfg.hold_travel_m]
        remaining = list(self._tracks)
        for observation in observations:
            world = np.array(observation.center_xy) @ rotation.T + pose
            world_heading = float(wrap_angle(yaw + observation.heading_rad))
            matches = []
            for track in remaining:
                if track.observation.role != observation.role:
                    continue
                angle = abs(float(wrap_angle(world_heading-track.heading_world)))
                distance = np.linalg.norm(world-track.center_world)
                # 前向截面随车前移，不把它误当成固定路口地标。
                if observation.role == 'FORWARD':
                    normal = np.array([-math.sin(world_heading), math.cos(world_heading)])
                    distance = abs(normal @ (world-track.center_world))
                if distance <= cfg.match_distance_m and angle <= math.radians(cfg.match_angle_deg):
                    matches.append((distance + angle, track))
            if matches:
                track = min(matches, key=lambda item: item[0])[1]
                remaining = [other for other in remaining if other is not track]
                if not observation.structural or not track.hits or track.last_ns != self._previous_ns:
                    track.hits, track.first_ns, track.confirmed = 0, stamp_ns, False
                track.center_world, track.heading_world = world, world_heading
                track.observation, track.last_ns, track.last_pose = observation, stamp_ns, pose.copy()
            else:
                track = _Track(self._next_id, world, world_heading, observation,
                               stamp_ns, stamp_ns, pose.copy())
                self._next_id += 1
                self._tracks.append(track)
            if observation.structural:
                track.hits += 1
                track.confirmed = (track.hits >= cfg.confirm_frames
                                   and (stamp_ns-track.first_ns)*1e-9 + 1e-9 >= cfg.confirm_time_s)
        snapshots = []
        for track in self._tracks:
            old = track.observation
            body = (track.center_world-pose) @ rotation
            relative_heading = float(wrap_angle(track.heading_world-yaw))
            age = (stamp_ns-track.last_ns)*1e-9
            relative_axis = float(wrap_angle(relative_heading-axis))
            side = 'FORWARD' if abs(relative_axis) < math.pi/4 else \
                ('BEHIND' if abs(relative_axis) > 3*math.pi/4 else
                 ('LEFT' if relative_axis > 0 else 'RIGHT'))
            obs = ExitObservation(tuple(body), relative_heading, old.width_m, old.depth_m,
                                  old.role, old.structural, old.wall_points, old.confidence,
                                  old.clipped, old.depth_visible_ratio)
            status = 'HELD' if age > 0 else ('CONFIRMED' if track.confirmed else
                                          ('TENTATIVE' if old.structural else 'UNVERIFIED'))
            snapshots.append(TrackedExit(track.track_id, obs, status, side, age, track.hits))
        self._previous_ns = stamp_ns
        eligible = [t for t in snapshots if t.state == 'CONFIRMED' and t.side != 'BEHIND']
        priorities = ('RIGHT', 'FORWARD', 'LEFT') if cfg.prefer_right else ('FORWARD', 'RIGHT', 'LEFT')
        preferred = min(eligible, key=lambda t: (priorities.index(t.side),
                        np.linalg.norm(t.observation.center_xy), t.track_id)) if eligible else None
        return ExitResult('EXIT_TRACKING' if eligible else 'EXIT_ACQUIRING',
                          '可见入口诊断；建议不参与控制，HELD/未确认入口不参与推荐',
                          tuple(snapshots), None if preferred is None else preferred.track_id,
                          'NONE' if preferred is None else preferred.side+'_AVAILABLE', axis,
                          self.sensor_offset)

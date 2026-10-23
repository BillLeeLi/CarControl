"""单个右转的行为仲裁、odom 路径和局部曲线扫掠检查（无 ROS 依赖）。"""

from dataclasses import dataclass
import math

import numpy as np

from process_node.perception import PlannerConfig, wrap_angle


def rotation(yaw):
    return np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])


@dataclass(frozen=True)
class TurnConfig:
    enabled: bool = True  # 仅在 corridor + closed_loop 模式接管右转
    approach_speed_mps: float = 0.10  # 接近路口的物理速度上限
    speed_mps: float = 0.10  # 转弯和出口对齐的物理速度上限
    activate_distance_m: float = 3.5  # 入口沿接近道路的前向距离上限
    min_radius_m: float = 0.70  # 规划允许的最小转弯半径，必须实车标定
    max_radius_m: float = 1.20  # 生成圆角参考时尝试的最大半径
    radius_samples: int = 4  # 在上述区间尝试的圆角半径数量
    lookahead_m: float = 0.65  # 沿已锁定路径选择跟踪点的距离
    check_distance_m: float = 0.80  # 每帧至少检查的前进弧长，不是完整路口可见性保证
    sample_step_m: float = 0.04  # 路径/扫掠采样间距；矩形额外膨胀覆盖采样间隙
    reaction_time_s: float = 0.30  # 估算停车距离所用的响应延迟
    braking_decel_mps2: float = 0.30  # 停车距离估计减速度，需实测核验
    exit_extension_m: float = 1.50  # 圆角末端/入口后继续延伸的对齐参考长度
    max_tracking_error_m: float = 0.55  # 偏离锁定路径则停止追踪，允许独立恢复重新检查
    min_angle_deg: float = 55.0  # 可接受的右出口相对接近道路转角下限
    max_angle_deg: float = 125.0  # 可接受的右出口相对接近道路转角上限
    hint_timeout_s: float = 0.50  # 疑似出口暂失多久后取消接近行为
    hint_context_s: float = 1.0  # 没有当前走廊时，接近轴参考的最长沿用时间
    max_update_gap_s: float = 0.50  # 大扫描间隔停车，不跨缺测段继续转弯
    max_duration_s: float = 60.0  # 单次接近/转弯的总时限，防止卡住后持续打方向
    max_travel_m: float = 7.0  # 单次行为从开始位置允许的最大位移
    exit_heading_tolerance_deg: float = 15.0  # 完成时新道路与锁定出口方向的容差
    exit_center_tolerance_m: float = 0.30  # 完成时相对新走廊中线的偏差容差
    exit_confirm_frames: int = 3  # 出口位置/航向/新走廊同时满足的连续帧数
    passed_exit_distance_m: float = 1.0  # 只抑制同位置同方向的已完成入口，非统一冷却时间

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError('turn_enabled 必须为布尔值')
        for name, value in vars(self).items():
            if name == 'enabled':
                continue
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'turn_{name} 必须为正有限值')
        for name in ('radius_samples', 'exit_confirm_frames'):
            if type(getattr(self, name)) is not int:
                raise ValueError(f'turn_{name} 必须为正整数')
        if (self.max_radius_m < self.min_radius_m
                or not 0 < self.min_angle_deg < self.max_angle_deg < 150
                or self.sample_step_m > min(self.lookahead_m, self.check_distance_m)/3
                or self.exit_heading_tolerance_deg >= 45):
            raise ValueError('turn_ 半径、角度或采样配置不合理')


@dataclass(frozen=True)
class TurnResult:
    phase: str  # CRUISE / APPROACH / TURN / EXIT_ALIGN / FAULT
    status: str  # OK、TURN_BLOCKED、TURN_FAULT 等诊断；与 PI/PD 的 warmup 状态分开
    reason: str  # 触发/等待/停止的依据
    active: bool = False  # 是否替代普通巡航参考；为真但 heading=None 表示停车
    heading_rad: float = None  # 检查通过的曲率对应的前瞻方向误差（左正，弧度）
    speed_mps: float = 0.0  # 物理目标速度上限，节点还会与全局速度目标取小
    exit_id: int = None  # 接近/锁定入口编号，转弯中不改选其他出口
    curvature_per_m: float = 0.0  # 纯追踪几何曲率，左正；不是 steer 或实测曲率
    path_body: tuple = ()  # odom 锁定参考在本帧车体坐标中的采样 (x,y,yaw)
    swept_body: tuple = ()  # 本帧实际检查的短程圆弧采样 (x,y,yaw)
    reset_corridor: bool = False  # 进入出口对齐时要求重新确认新走廊
    recoverable: bool = False  # 仅路径问题/短暂输入中断可交给独立恢复行为


def arc_poses(curvature, distance, step):
    """单位弧长积分的圆弧；第一点为当前车体原点。"""
    s = np.linspace(0., distance, math.ceil(distance/step)+1)
    if abs(curvature) < 1e-8:
        return np.column_stack((s, np.zeros_like(s), np.zeros_like(s)))
    a = curvature*s
    return np.column_stack((np.sin(a)/curvature, (1-np.cos(a))/curvature, a))


def check_swept_path(data, poses, planner, sensor_offset=(0., 0.), *, check_visibility=True):
    """检查沿圆弧运动的整车矩形，不用一个远处的直线射线代替转弯。

    真实点全部参与碰撞检查；每个矩形覆盖到的角箱必须有足够可见深度。
    采样间隙用最大平移/旋转位移膨胀，未知/插值不可通行，+inf 只用受限深度。
    构造长参考时可仅排除已知障碍；实际控制的短弧始终同时检查可见性。
    这仍是规划模型检查，不能在未标定的实车上保证实际转向响应。
    """
    poses = np.asarray(poses, dtype=float)
    if (poses.ndim != 2 or poses.shape[1] != 3 or len(poses) < 2
            or not np.isfinite(poses).all()):
        return False, '扫掠路径无效'
    if data.too_close.any() or data.valid_ratio < planner.min_scan_finite_ratio or not data.valid.any():
        return False, '近距离异常或真实测距比例不足'
    sensor = np.asarray(sensor_offset)
    points = np.column_stack((data.ranges[data.valid]*np.cos(data.angles[data.valid]),
                              data.ranges[data.valid]*np.sin(data.angles[data.valid]))) + sensor
    # 尺寸相对雷达测量，扫掠位姿却在 base_link 原点；矩形必须平移到同一坐标系。
    padded_low, padded_high = planner.footprint_bounds(sensor, planner.safety_margin_m)
    radius = float(np.linalg.norm(np.maximum(np.abs(padded_low), np.abs(padded_high))))
    # 任一相邻采样之间，车体任一点到较近采样位姿的位移不超过此上界。
    inflation = .5*np.max(np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
                           + radius*np.abs(wrap_angle(np.diff(poses[:, 2]))))
    low, high = padded_low-inflation, padded_high+inflation
    rays = np.column_stack((np.cos(data.bin_angles), np.sin(data.bin_angles)))
    # 膨胀相邻角箱深度取小，防止墙端/未知角箱在分箱边缘穿漏。
    near = (np.arange(len(rays))[:, None]+[-1, 0, 1]) % len(rays)
    depth = np.min(data.bin_ranges[near], axis=1)
    known = np.isfinite(depth) & ~data.inferred[near].any(axis=1)
    for x, y, yaw in poses:
        rot = rotation(yaw)
        local = (points-[x, y]) @ rot
        if ((local >= low).all(axis=1) & (local <= high).all(axis=1)).any():
            return False, '曲线路径的整车包络内存在真实障碍'
        if not check_visibility:
            continue
        origin = (sensor-[x, y]) @ rot
        directions = rays @ rot
        # 射线与矩形的 slab 相交：比较每条穿过矩形的射线所需的最远可见距离。
        parallel = np.abs(directions) < 1e-12
        safe_directions = np.where(parallel, 1., directions)
        a, b = (low-origin)/safe_directions, (high-origin)/safe_directions
        enter, leave = np.minimum(a, b), np.maximum(a, b)
        inside = (origin >= low) & (origin <= high)
        enter = np.where(parallel, np.where(inside, -np.inf, np.inf), enter)
        leave = np.where(parallel, np.where(inside, np.inf, -np.inf), leave)
        near_t, far_t = enter.max(axis=1), leave.min(axis=1)
        intersect = far_t >= np.maximum(near_t, 0.)
        # 角箱中心以外的小偏差也留出距离余量；不会把墙后区域当自由。
        padding = np.maximum(far_t, 0)*data.bin_step
        if np.any(intersect & (~known | (depth <= far_t+padding))):
            return False, '曲线整车包络进入未知、遮挡或无回波规划上限之外'
    return True, '局部曲线整车包络的真实障碍和可见性检查通过'


def fillet_path(position, incoming_heading, center_error, observation, radius, config):
    """接近中线与出口轴相交处构造相切圆角；观测入口不是立即朝向的目标。"""
    u = np.array([math.cos(incoming_heading), math.sin(incoming_heading)])
    h = observation.heading_rad
    v = np.array([math.cos(h), math.sin(h)])
    angle = -float(wrap_angle(h-incoming_heading))
    if not math.radians(config.min_angle_deg) <= angle <= math.radians(config.max_angle_deg):
        return None
    line_origin = np.asarray(position) + center_error*np.array([-u[1], u[0]])
    gate = np.asarray(observation.center_xy)
    try:
        intersection = line_origin + np.linalg.solve(np.column_stack((u, -v)), gate-line_origin)[0]*u
    except np.linalg.LinAlgError:
        return None
    tangent = radius*math.tan(angle/2)
    start, end = intersection-tangent*u, intersection+tangent*v
    ahead = (start-line_origin) @ u
    # 已经驶过起弯点时不补一条倒退路径，也不突然原地拐弯。
    if ahead < -.15:
        return None
    s = np.linspace(min(0., ahead), ahead, max(2, math.ceil(max(0., ahead)/config.sample_step_m)+1))
    straight = np.column_stack((line_origin+s[:, None]*u, np.full(len(s), incoming_heading)))
    arc = arc_poses(-1/radius, radius*angle, config.sample_step_m)
    arc[:, :2] = arc[:, :2] @ rotation(incoming_heading).T + start
    arc[:, 2] += incoming_heading
    length = max(config.exit_extension_m, (gate-end) @ v + config.exit_extension_m)
    tail_s = np.linspace(0., length, math.ceil(length/config.sample_step_m)+1)
    tail = np.column_stack((end+tail_s[:, None]*v, np.full(len(tail_s), incoming_heading-angle)))
    path = np.concatenate((straight[:-1], arc, tail[1:]))
    return path, len(straight)-1, len(straight)+len(arc)-2


class RightTurnBehavior:
    """一个入口一条 odom 路径；障碍只会使其停车，不会临时改选另一个路口。"""

    def __init__(self, config=TurnConfig(), planner=None, sensor_offset=(0., 0.)):
        self.config = config
        self.planner = PlannerConfig() if planner is None else planner
        self.sensor_offset = sensor_offset
        self.reset()

    def reset(self):
        self.phase = 'CRUISE'
        self._last_ns = None
        self._started_ns = None
        self._start_pose = None
        self._hint_ns = None
        self._axis_context = None
        self._path = None
        self._exit_id = None
        self._gate_world = None
        self._out_heading = None
        self._progress = 0
        self._align_hits = 0
        self._passed = []
        self._fault = ''
        self._recoverable = False

    def invalidate(self, reason, recoverable=False):
        """保留首次原因；硬异常可升级软故障，但不能反向降级。"""
        if self.phase == 'FAULT':
            if self._recoverable and not recoverable:
                self._fault += '；后续硬异常：'+reason
            self._recoverable = self._recoverable and recoverable
            return
        if self.phase != 'CRUISE':
            self.phase, self._fault = 'FAULT', reason
            self._recoverable = recoverable
        else:
            self._axis_context, self._last_ns = None, None
            self._passed = []

    def _result(self, status, reason, **kwargs):
        return TurnResult(self.phase, status, reason, self.phase != 'CRUISE',
                          exit_id=self._exit_id, recoverable=self._recoverable, **kwargs)

    @property
    def recovery_goal(self):
        """只公开已确认道路出口，疑似墙端不能成为恢复转向目标。"""
        if self.phase == 'CRUISE' or self._gate_world is None:
            return None
        return (tuple(self._gate_world), self._out_heading, self._exit_id)

    def finish_recovery(self):
        """恢复模块已验证整车越过入口且新道路对齐后，交还巡航。"""
        if self.phase != 'CRUISE' and self._gate_world is not None:
            self._passed.append((self._gate_world.copy(), self._out_heading))
            self._passed = self._passed[-8:]
        self.phase, self._path, self._exit_id = 'CRUISE', None, None
        self._gate_world = self._out_heading = None
        self._fault, self._recoverable = '', False
        self._axis_context, self._last_ns = None, None
        return self._result('OK', '局部恢复完成，交还巡航')

    def update(self, data, feedback, stamp_ns, corridor, exits, cruise_angle=None):
        cfg = self.config
        if not cfg.enabled:
            self.reset()
            return self._result('DISABLED', '右转行为已关闭')
        if self.phase == 'FAULT':
            return self._result('TURN_FAULT', self._fault)
        if not feedback.valid or data.too_close.any() or not data.valid.any() \
                or data.valid_ratio < self.planner.min_scan_finite_ratio:
            transient = (not feedback.valid and feedback.status in
                         ('STALE_ODOM', 'UNALIGNED_ODOM', 'WAITING_ODOM')
                         and not data.too_close.any() and data.valid.any()
                         and data.valid_ratio >= self.planner.min_scan_finite_ratio)
            self.invalidate('输入或运动反馈失效，不能沿历史路径继续转弯', transient)
            return self._result('TURN_FAULT' if self.phase == 'FAULT' else 'TURN_WAITING',
                                '等待有效雷达和独立运动反馈')
        if self._last_ns is not None and (stamp_ns <= self._last_ns
                or (stamp_ns-self._last_ns)*1e-9 > cfg.max_update_gap_s):
            self.invalidate('转弯期间扫描重复、回跳或间隔过大', stamp_ns > self._last_ns)
            self._last_ns = stamp_ns
            return self._result('TURN_FAULT' if self.phase == 'FAULT' else 'TURN_WAITING',
                                '扫描时间异常')
        self._last_ns = stamp_ns
        state = feedback.state
        pose = np.array([state.x_m, state.y_m])
        yaw, rot = state.yaw_rad, rotation(state.yaw_rad)
        if corridor is not None and corridor.usable and corridor.status == 'CORRIDOR_TRACKING':
            self._axis_context = (yaw+corridor.heading_rad,
                                  pose+corridor.lateral_error_m*np.array(
                                      [-math.sin(yaw+corridor.heading_rad), math.cos(yaw+corridor.heading_rad)]),
                                  stamp_ns)
        tracks = () if exits is None else exits.tracks
        # 完成过的同一入口即使重新编号也不再触发；不屏蔽附近其他方向的出口。
        def eligible(track):
            obs = track.observation
            world = pose+np.array(obs.center_xy) @ rot.T
            return (track.side == 'RIGHT' and track.state in ('CONFIRMED', 'SUSPECTED')
                    and obs.role in ('SIDE', 'CORNER_HINT')
                    and 0 < obs.center_xy[0] < cfg.activate_distance_m
                    and not any(np.linalg.norm(world-old[0]) < cfg.passed_exit_distance_m
                                and abs(float(wrap_angle(yaw+obs.heading_rad-old[1]))) < math.pi/4
                                for old in self._passed))
        if self.phase == 'CRUISE':
            choices = [t for t in tracks if eligible(t)]
            if choices and self._axis_context is not None:
                chosen = min(choices, key=lambda t: np.linalg.norm(t.observation.center_xy))
                self.phase, self._exit_id = 'APPROACH', chosen.track_id
                self._progress, self._align_hits = 0, 0
                self._hint_ns, self._started_ns, self._start_pose = stamp_ns, stamp_ns, pose.copy()
        if self.phase == 'CRUISE':
            return self._result('OK', '没有待执行的右转，沿走廊巡航')
        if ((stamp_ns-self._started_ns)*1e-9 > cfg.max_duration_s
                or np.linalg.norm(pose-self._start_pose) > cfg.max_travel_m):
            self.invalidate('单次转弯超过时间或位移上限')
            return self._result('TURN_FAULT', self._fault)
        if self._path is None:
            # 疑似出口只允许沿可信接近轴前进，不因斜向 inf 直接打大方向。
            matching = next((t for t in tracks if t.track_id == self._exit_id
                             and t.state != 'HELD'), None)
            if matching is not None:
                self._hint_ns = stamp_ns
            if matching is None and (stamp_ns-self._hint_ns)*1e-9 > cfg.hint_timeout_s:
                self.phase, self._exit_id = 'CRUISE', None
                return self._result('OK', '疑似开口消失，取消接近并重新判断')
            axis, line_point, context_ns = self._axis_context
            if (stamp_ns-context_ns)*1e-9 > cfg.hint_context_s:
                return self._result('TURN_APPROACH_WAITING', '接近道路参考过期，等待出口确认')
            if matching is not None and matching.state == 'CONFIRMED' \
                    and matching.observation.role == 'SIDE' and matching.observation.structural:
                # 即使固定圆角无法接入，也可把这个已确认出口交给受限恢复行为。
                self._gate_world = pose+np.asarray(matching.observation.center_xy) @ rot.T
                self._out_heading = yaw+matching.observation.heading_rad
                # 用当前车体坐标建立圆角，再一次性变换到 odom；之后不随出口抖动重画。
                relative_axis = float(wrap_angle(axis-yaw))
                center_error = (line_point-pose) @ np.array([-math.sin(axis), math.cos(axis)])
                options = []
                for radius in np.linspace(cfg.min_radius_m, cfg.max_radius_m, cfg.radius_samples):
                    built = fillet_path((0., 0.), relative_axis, center_error,
                                        matching.observation, radius, cfg)
                    if built is None:
                        continue
                    path, start, end = built
                    # 长参考不能穿过当前已知障碍；未观察到的后半段留待接近后逐帧检查。
                    if not check_swept_path(data, path, self.planner, self.sensor_offset,
                                            check_visibility=False)[0]:
                        continue
                    # 先选当前实际跟踪圆弧可检查通过的参考，不能靠任意一个侧向开口强行转。
                    heading, curvature, _, _ = self._follow(path, np.zeros(2), 0., 0)
                    if heading is None:
                        continue
                    swept = self._sweep(curvature, feedback)
                    safe, _ = check_swept_path(data, swept, self.planner, self.sensor_offset)
                    if safe:
                        options.append((radius, path, start, end))
                if options:
                    _, path, self._arc_start, self._arc_end = options[-1]
                    self._path = path.copy()
                    self._path[:, :2] = path[:, :2] @ rot.T+pose
                    self._path[:, 2] = path[:, 2]+yaw
                    self._gate_world = pose+np.asarray(matching.observation.center_xy) @ rot.T
                    self._out_heading = yaw+matching.observation.heading_rad
            if self._path is None:
                if matching is not None and matching.state == 'CONFIRMED':
                    return self._result('TURN_NO_PATH', '确认出口尚无可接入的圆角路径，停车等待')
                # 精确当前巡航参考优先；无墙时仅短时沿历史接近轴低速观察。
                heading = cruise_angle
                if heading is None:
                    local = (line_point-pose) @ rot
                    heading = float(wrap_angle(axis-yaw))+math.atan2(local[1], cfg.lookahead_m)
                if abs(heading) > math.radians(30):
                    return self._result('TURN_APPROACH_WAITING', '疑似出口接近误差过大，停止')
                curvature = float(np.clip(2*math.sin(heading)/cfg.lookahead_m,
                                          -1/cfg.min_radius_m, 1/cfg.min_radius_m))
                swept = self._sweep(curvature, feedback)
                safe, reason = check_swept_path(data, swept, self.planner, self.sensor_offset)
                return self._result('OK' if safe else 'TURN_BLOCKED', reason,
                                    heading_rad=math.atan(curvature*cfg.lookahead_m/2) if safe else None,
                                    speed_mps=cfg.approach_speed_mps, curvature_per_m=curvature,
                                    swept_body=tuple(map(tuple, swept)))
        body_path = self._path.copy()
        body_path[:, :2] = (self._path[:, :2]-pose) @ rot
        body_path[:, 2] = wrap_angle(self._path[:, 2]-yaw)
        heading, curvature, progress, error = self._follow(self._path, pose, yaw, self._progress)
        self._progress = progress
        new_corridor = (self.phase == 'EXIT_ALIGN' and corridor is not None and corridor.usable
                        and corridor.status == 'CORRIDOR_TRACKING'
                        and abs(float(wrap_angle(yaw+corridor.heading_rad-self._out_heading)))
                        < math.radians(cfg.exit_heading_tolerance_deg))
        if new_corridor:
            # 可见入口截面中心未必就是真实新道路中线；对齐阶段用新双墙回中。
            tangent = np.array([math.cos(corridor.heading_rad), math.sin(corridor.heading_rad)])
            normal = np.array([-tangent[1], tangent[0]])
            # 对齐用转弯的近前瞻；直接复用巡航 1.5 m 远点会拖长出口回中时间。
            point = cfg.lookahead_m*tangent+corridor.lateral_error_m*normal
            curvature = float(np.clip(2*point[1]/(point @ point),
                                      -1/cfg.min_radius_m, 1/cfg.min_radius_m))
            heading = math.atan(curvature*cfg.lookahead_m/2)
        if (error > cfg.max_tracking_error_m or heading is None) and not new_corridor:
            self.invalidate('偏离锁定路径或目标已到车后，停止并检查车辆转向标定', True)
            return self._result('TURN_FAULT', self._fault, path_body=tuple(map(tuple, body_path)))
        reset_corridor = False
        if self.phase == 'APPROACH' and progress >= max(0, self._arc_start-2):
            self.phase = 'TURN'
        out_error = abs(float(wrap_angle(self._out_heading-yaw)))
        if self.phase == 'TURN' and progress >= self._arc_end-2 and out_error < math.radians(25):
            self.phase, self._align_hits = 'EXIT_ALIGN', 0
            reset_corridor = True  # 下一帧起拟合新道路，旧巡航模型不能充当完成依据。
        if self.phase == 'EXIT_ALIGN' and not reset_corridor:
            outbound = np.array([math.cos(self._out_heading), math.sin(self._out_heading)])
            body_low, body_high = self.planner.footprint_bounds(
                self.sensor_offset, self.planner.safety_margin_m)
            # 雷达更靠近车头；完成判据须等车尾及裕量越过入口，不能只用较短的车头距离。
            # 把完整矩形投影到出口方向，也计入尚未完全回正时车角的后向投影。
            axis_body = outbound @ rot
            back_projection = np.where(axis_body >= 0, body_low, body_high) @ axis_body
            in_new_road = (pose-self._gate_world) @ outbound + back_projection > 0
            aligned = (in_new_road and out_error < math.radians(cfg.exit_heading_tolerance_deg)
                       and corridor is not None and corridor.status == 'CORRIDOR_TRACKING'
                       and corridor.usable and abs(corridor.heading_rad) < math.radians(cfg.exit_heading_tolerance_deg)
                       and abs(float(wrap_angle(yaw+corridor.heading_rad-self._out_heading)))
                       < math.radians(cfg.exit_heading_tolerance_deg)
                       and abs(corridor.lateral_error_m) < cfg.exit_center_tolerance_m)
            self._align_hits = self._align_hits+1 if aligned else 0
            if self._align_hits >= cfg.exit_confirm_frames:
                self._passed.append((self._gate_world.copy(), self._out_heading))
                self._passed = self._passed[-8:]
                self.phase, self._path, self._exit_id = 'CRUISE', None, None
                return self._result('OK', '位置、出口方向和新走廊连续确认，交还巡航')
        # 新障碍/遮挡只暂停同一条路径，不切换出口；总时限仍继续计时。
        swept = self._sweep(curvature, feedback)
        safe, reason = check_swept_path(data, swept, self.planner, self.sensor_offset)
        return self._result('OK' if safe else 'TURN_BLOCKED', reason,
                            heading_rad=heading if safe else None,
                            speed_mps=cfg.approach_speed_mps if self.phase == 'APPROACH' else cfg.speed_mps,
                            curvature_per_m=curvature, path_body=tuple(map(tuple, body_path)),
                            swept_body=tuple(map(tuple, swept)), reset_corridor=reset_corridor)

    def _sweep(self, curvature, feedback):
        speed = abs(getattr(feedback.state, 'speed_mps', 0.))
        cfg = self.config
        distance = max(cfg.check_distance_m, speed*cfg.reaction_time_s
                       + speed*speed/(2*cfg.braking_decel_mps2))
        return arc_poses(curvature, distance, cfg.sample_step_m)

    def _follow(self, path, pose, yaw, previous):
        cfg = self.config
        distances = np.linalg.norm(path[previous:, :2]-pose, axis=1)
        progress = previous+int(np.argmin(distances))
        remaining = path[progress:, :2]
        lengths = np.r_[0., np.cumsum(np.linalg.norm(np.diff(remaining, axis=0), axis=1))]
        index = progress+min(int(np.searchsorted(lengths, cfg.lookahead_m)), len(remaining)-1)
        point = (path[index, :2]-pose) @ rotation(yaw)
        if point[0] <= .02 or np.linalg.norm(point) < .10:
            return None, 0., progress, float(distances.min())
        curvature = float(np.clip(2*point[1]/(point @ point), -1/cfg.min_radius_m, 1/cfg.min_radius_m))
        # 曲率截断后同步收回方向误差，不能检查一条缓弧却要求极大的方向角。
        heading = math.atan(curvature*cfg.lookahead_m/2)
        return heading, curvature, progress, float(distances.min())

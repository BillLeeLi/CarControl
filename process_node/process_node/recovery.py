"""受约束的局部恢复；只在正常参考失效时搜索短弧，不替代道路分支选择。"""

from dataclasses import dataclass
import math
import time

import numpy as np

from process_node.motion import wrap_angle
from process_node.turning import arc_poses, check_swept_path, rotation


@dataclass(frozen=True)
class RecoveryConfig:
    enabled: bool = True  # 仅 corridor + closed_loop 使用
    speed_mps: float = 0.06  # 恢复请求速度上限；仍与全局目标取小
    stop_speed_mps: float = 0.05  # 开始修正前须确认实际速度已足够低
    confirm_frames: int = 3  # 连续输入、候选或交接条件的确认帧数
    confirm_time_s: float = 0.20  # 连续确认最短时长，不依赖固定帧率
    check_distance_m: float = 0.60  # 最少检查弧长，另计停车距离
    curvature_samples: int = 13  # [-1/min_radius, +1/min_radius] 的候选数，包含直行
    max_heading_deviation_deg: float = 65.0  # 候选末端航向偏离引导方向的上限
    guide_timeout_s: float = 1.0  # 无当前真实墙线时，近期道路引导最多沿用多久
    guide_travel_m: float = 0.35  # 无新道路观测时允许沿用引导的最大位移
    max_input_gap_s: float = 1.0  # 超过此扫描间隔不再自动恢复
    max_duration_s: float = 20.0  # 同一组恢复尝试的总时间预算，等待也计时
    max_travel_m: float = 0.80  # 恢复期间累计路程上限，来回移动也累计
    max_attempts: int = 3  # 正常行驶尚未稳定时最多重新进入恢复的次数
    normal_reset_time_s: float = 1.0  # 正常控制连续稳定后才刷新尝试预算
    progress_timeout_s: float = 4.0  # 执行修正时长期不接近目标则停车
    min_progress_m: float = 0.03  # 上述时间内至少减少的目标距离

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError('recovery_enabled 必须为布尔值')
        for name, value in vars(self).items():
            if name != 'enabled' and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'recovery_{name} 必须为正有限值')
        for name in ('confirm_frames', 'curvature_samples', 'max_attempts'):
            if type(getattr(self, name)) is not int:
                raise ValueError(f'recovery_{name} 必须为正整数')
        if self.curvature_samples < 3 or self.curvature_samples % 2 != 1:
            raise ValueError('recovery_curvature_samples 必须为至少 3 的奇数')
        if not 0 < self.max_heading_deviation_deg < 90:
            raise ValueError('recovery_max_heading_deviation_deg 必须小于 90 度')


@dataclass(frozen=True)
class RecoveryResult:
    phase: str  # IDLE / WAIT / CONFIRM / MOVE / HANDOFF / FAULT
    status: str  # OK 才能执行短弧；其余活动状态都停车
    reason: str  # 当前动作或等待原因
    active: bool = False  # 是否覆盖正常行为
    heading_rad: float = None  # 给 PD 的方向误差，左正、弧度
    speed_mps: float = 0.0  # 物理请求速度上限
    curvature_per_m: float = 0.0  # 本帧通过整车检查的规划曲率
    swept_body: tuple = ()  # 以当前 base_link 为原点的短弧位姿
    handoff: bool = False  # 已连续确认可交还正常巡航，本帧仍停止
    cause: str = ''  # 本轮第一次进入恢复的原因，后续等待不会覆盖


class LocalRecoveryBehavior:
    """停车确认 → 安全短弧 → 连续确认正常参考；没有可信引导时只等待。"""

    TRIGGERS = {'CRUISE_BLOCKED', 'CORRIDOR_LOST', 'CORRIDOR_OUT_OF_RANGE',
                'NO_CLEAR_PATH', 'NO_OBSERVED_GAP'}
    TURN_TRIGGERS = {'TURN_BLOCKED', 'TURN_NO_PATH', 'TURN_APPROACH_WAITING'}

    def __init__(self, config, planner, turn_config, sensor_offset=(0., 0.)):
        self.config, self.planner, self.turn_config = config, planner, turn_config
        self.sensor_offset = sensor_offset
        if config.check_distance_m < 3*turn_config.sample_step_m:
            raise ValueError('恢复检查距离需至少包含三个扫掠采样间隔')
        self.reset()

    def reset(self):
        self.phase = 'IDLE'
        self._guide = None
        self._last_ns = None
        self._last_received = None
        self._last_pose = None
        self._started_ns = None
        self._attempts = 0
        self._travel = 0.
        self._normal_since = None
        self._hits, self._since = 0, None
        self._handoff_hits, self._handoff_since = 0, None
        self._previous_k = None
        self._goal = None
        self._turn_goal = None
        self._progress_ns, self._best_distance = None, None
        self._moved = False
        self.cause, self._fault = '', ''

    @property
    def active(self):
        return self.phase != 'IDLE'

    def interrupt(self, reason, hard=False):
        """看门狗只使其等待；位姿/时间不可信时取消自动运动并锁存。"""
        if self.phase == 'FAULT':
            return
        self._normal_since = None
        if hard:
            if self.active:
                self.phase, self._fault = 'FAULT', reason
            else:
                self.reset()
            return
        if self.active:
            self.phase = 'WAIT'
            self._clear_confirmation()

    def _clear_confirmation(self):
        self._hits, self._since = 0, None
        self._handoff_hits, self._handoff_since = 0, None
        self._previous_k = None
        self._progress_ns = None  # 暂停期间不累计“执行却无进展”；总时间/路程预算不重置。

    def _result(self, status, reason, **kwargs):
        return RecoveryResult(self.phase, status, reason, self.active,
                              cause=self.cause, **kwargs)

    def _fault_result(self, reason):
        self.phase, self._fault = 'FAULT', reason
        return self._result('RECOVERY_FAULT', reason)

    def _confirmed(self, hits, since, stamp_ns):
        return hits >= self.config.confirm_frames and (
            stamp_ns-since)*1e-9 >= self.config.confirm_time_s-1e-9

    def _remember_road(self, corridor, state, stamp_ns):
        if (corridor is not None and corridor.usable
                and corridor.status in ('CORRIDOR_TRACKING', 'CORRIDOR_SINGLE_WALL')):
            h = state.yaw_rad+corridor.heading_rad
            normal = np.array([-math.sin(h), math.cos(h)])
            pose = np.array([state.x_m, state.y_m])
            self._guide = (pose+corridor.lateral_error_m*normal, h, stamp_ns, pose)

    def _goal_for(self, state, stamp_ns):
        pose = np.array([state.x_m, state.y_m])
        if self._turn_goal is not None:
            gate, h, _ = self._turn_goal
            return np.asarray(gate)+self.turn_config.exit_extension_m*np.array(
                [math.cos(h), math.sin(h)]), h
        if self._guide is None:
            return None
        point, h, ns, observed_pose = self._guide
        if ((stamp_ns-ns)*1e-9 > self.config.guide_timeout_s
                or np.linalg.norm(pose-observed_pose) > self.config.guide_travel_m):
            return None
        tangent = np.array([math.cos(h), math.sin(h)])
        # 历史中线在 odom 中固定，按当前位置投影，避免只旋转不补偿平移。
        return point+((pose-point) @ tangent+self.turn_config.lookahead_m)*tangent, h

    def _can_handoff(self, corridor, cruise_angle, state):
        if (corridor is None or corridor.status != 'CORRIDOR_TRACKING'
                or not corridor.usable or cruise_angle is None):
            return False
        cfg = self.turn_config
        if (abs(corridor.heading_rad) > math.radians(cfg.exit_heading_tolerance_deg)
                or abs(corridor.lateral_error_m) > cfg.exit_center_tolerance_m):
            return False
        if self._turn_goal is None:
            return True
        gate, h, _ = self._turn_goal
        if abs(wrap_angle(state.yaw_rad+corridor.heading_rad-h)) > math.radians(
                cfg.exit_heading_tolerance_deg):
            return False  # 未进入所选出口，不能因原走廊仍可直行而放弃右转。
        axis = np.array([math.cos(h), math.sin(h)])
        body_axis = axis @ rotation(state.yaw_rad)
        low, high = self.planner.footprint_bounds(self.sensor_offset, self.planner.safety_margin_m)
        tail = np.where(body_axis >= 0, low, high) @ body_axis
        return (np.array([state.x_m, state.y_m])-gate) @ axis+tail > 0

    def update(self, data, feedback, stamp_ns, corridor, cruise_angle, plan_status,
               turn, turn_goal=None, received_at=None):
        cfg = self.config
        if not cfg.enabled:
            self.reset()
            return self._result('DISABLED', '局部恢复已关闭')
        if self.phase == 'FAULT':
            return self._result('RECOVERY_FAULT', self._fault)
        # 硬转弯故障不可被恢复模块覆盖，包括 odom 重置及行为预算耗尽。
        if turn.active and turn.status == 'TURN_FAULT' and not turn.recoverable:
            if self.active:
                return self._fault_result('原行为存在不可自动恢复的故障：'+turn.reason)
            return self._result('IDLE', '原行为硬故障，保持停车')
        if (data is None or data.too_close.any() or not data.valid.any()
                or data.valid_ratio < self.planner.min_scan_finite_ratio):
            self.interrupt('雷达数据不可用于恢复', hard=True)
            return self._result('RECOVERY_FAULT' if self.phase == 'FAULT' else 'RECOVERY_WAITING',
                                '雷达无效，禁止恢复运动')
        if not feedback.valid:
            self.interrupt('独立反馈异常：'+feedback.status, hard=feedback.status not in
                           ('STALE_ODOM', 'UNALIGNED_ODOM', 'WAITING_ODOM'))
            return self._result('RECOVERY_FAULT' if self.phase == 'FAULT' else 'RECOVERY_WAITING',
                                '等待有效且对齐的独立反馈')
        state = feedback.state
        pose = np.array([state.x_m, state.y_m])
        if not np.isfinite([*pose, state.yaw_rad, state.speed_mps]).all():
            return self._fault_result('恢复反馈包含非有限位姿或速度')
        received = time.monotonic() if received_at is None else received_at
        real_gap = None if self._last_received is None else received-self._last_received
        if real_gap is not None and (real_gap <= 0 or real_gap > cfg.max_input_gap_s):
            if self.active or (turn.active and turn.recoverable):
                return self._fault_result('真实有效输入接收中断超过恢复上限或接收时间异常')
            self._guide = None
            self._normal_since = None
        self._last_received = received
        if self._last_ns is not None:
            gap = (stamp_ns-self._last_ns)*1e-9
            if gap <= 0 or gap > cfg.max_input_gap_s:
                if self.active or (turn.active and turn.recoverable):
                    return self._fault_result('恢复时间回跳、重复或输入中断超过恢复上限')
                self._guide = None
                self._normal_since = None
            elif gap > self.turn_config.max_update_gap_s:
                self.interrupt('输入间隔较大，重新停车确认')
        if self.active and self._last_pose is not None:
            self._travel += float(np.linalg.norm(pose-self._last_pose))
        self._last_ns, self._last_pose = stamp_ns, pose
        self._remember_road(corridor, state, stamp_ns)

        normal = (turn.status == 'OK' and turn.heading_rad is not None) if turn.active \
            else cruise_angle is not None
        trigger = (turn.recoverable if turn.status == 'TURN_FAULT' else
                   turn.status in self.TURN_TRIGGERS) if turn.active \
            else plan_status in self.TRIGGERS
        if not self.active:
            if normal:
                if self._normal_since is None:
                    self._normal_since = stamp_ns
                if (stamp_ns-self._normal_since)*1e-9 >= cfg.normal_reset_time_s:
                    self._started_ns, self._attempts, self._travel = None, 0, 0.
                return self._result('IDLE', '正常行为可执行')
            self._normal_since = None
            if not trigger:
                return self._result('IDLE', '当前问题不属于局部恢复范围')
            self._turn_goal = turn_goal if turn.active else None
            if self._goal_for(state, stamp_ns) is None:
                # 只有疑似开口、从未建立道路参考时，保持原行为停车，
                # 不启动一次注定无法动作的恢复并把后续正常道路锁死。
                return self._result('RECOVERY_UNAVAILABLE', '没有可信恢复引导，保留原行为等待')
            self._attempts += 1
            if self._attempts > cfg.max_attempts:
                return self._fault_result('恢复重试次数耗尽，需人工检查')
            self.phase, self.cause = 'WAIT', turn.reason if turn.active else plan_status
            self._started_ns = stamp_ns if self._started_ns is None else self._started_ns
            self._progress_ns, self._best_distance = None, None
            self._moved = False
            self._clear_confirmation()
            return self._result('RECOVERY_WAITING', '先停车，再确认环境与实际速度')

        if ((stamp_ns-self._started_ns)*1e-9 > cfg.max_duration_s
                or self._travel > cfg.max_travel_m):
            return self._fault_result('局部恢复超过总时间或累计路程预算')
        # 尚未执行恢复动作时，原行为自行恢复便直接退出，避免干预短暂障碍等待。
        if normal and self.phase in ('WAIT', 'CONFIRM') and turn.status != 'TURN_FAULT':
            self.phase, self._normal_since = 'IDLE', stamp_ns
            return self._result('IDLE', '原行为已恢复，取消局部修正')
        if self._can_handoff(corridor, cruise_angle, state):
            self._handoff_since = stamp_ns if self._handoff_since is None else self._handoff_since
            self._handoff_hits += 1
            self.phase = 'HANDOFF'
            if self._confirmed(self._handoff_hits, self._handoff_since, stamp_ns):
                result = self._result('RECOVERY_COMPLETE', '可靠新走廊与安全巡航候选连续确认', handoff=True)
                self.phase, self._normal_since = 'IDLE', stamp_ns
                return result
            return self._result('RECOVERY_CONFIRMING', '停车连续确认交还巡航条件')
        self._handoff_hits, self._handoff_since = 0, None
        if self.phase == 'HANDOFF':
            self.phase = 'WAIT'
            self._clear_confirmation()
        goal = self._goal_for(state, stamp_ns)
        if goal is None:
            if not self._moved and self._turn_goal is None:
                self.phase = 'IDLE'
                self._clear_confirmation()
                return self._result('RECOVERY_UNAVAILABLE', '起步前道路引导已过期，取消恢复并保留原行为等待')
            self.phase = 'WAIT'
            self._clear_confirmation()
            return self._result('RECOVERY_NO_GUIDE', '缺少当前/近期道路或已确认出口引导，仅停车等待')
        if self.phase != 'MOVE' and abs(state.speed_mps) > cfg.stop_speed_mps:
            self.phase = 'WAIT'
            self._clear_confirmation()
            return self._result('RECOVERY_WAITING', '等待实际速度降到恢复起步门槛')
        goal_world, axis = goal
        local_goal = (goal_world-pose) @ rotation(state.yaw_rad)
        if np.linalg.norm(local_goal) < .10:
            return self._fault_result('已到局部引导末端但不能建立正常参考')
        distance = max(cfg.check_distance_m, abs(state.speed_mps)*self.turn_config.reaction_time_s
                       + state.speed_mps**2/(2*self.turn_config.braking_decel_mps2))
        limit = 1/self.turn_config.min_radius_m
        ideal = float(np.clip(2*local_goal[1]/(local_goal @ local_goal), -limit, limit))
        curvatures = list(np.linspace(-limit, limit, cfg.curvature_samples))+[ideal]
        if self._previous_k is not None:
            curvatures.append(self._previous_k)
        choices = []
        goal_bearing = math.atan2(local_goal[1], local_goal[0])
        for k in curvatures:
            poses = arc_poses(k, distance, self.turn_config.sample_step_m)
            end = poses[-1]
            if abs(wrap_angle(end[2]-goal_bearing)) > math.radians(cfg.max_heading_deviation_deg):
                continue
            # 必须朝可信目标取得进展，不能只因某个侧面更空旷就驶向那里。
            remaining = float(np.linalg.norm(local_goal-end[:2]))
            if remaining >= np.linalg.norm(local_goal)-.01:
                continue
            safe, _ = check_swept_path(data, poses, self.planner, self.sensor_offset)
            if not safe:
                continue
            previous = 0. if self._previous_k is None else self._previous_k
            # 到引导点的进展为主；航向和曲率平滑为次，不能把回中动作全部压成直行。
            score = remaining+.06*abs(wrap_angle(state.yaw_rad+end[2]-axis)) \
                + .04*abs(k-previous)+.02*abs(k)
            choices.append((score, float(k), poses))
        if not choices:
            self.phase = 'WAIT'
            self._clear_confirmation()
            return self._result('RECOVERY_BLOCKED', '没有通过整车障碍/可见性检查且朝目标前进的短弧')
        _, k, poses = min(choices, key=lambda c: c[0])
        if self.phase != 'MOVE':
            if self._previous_k is not None and abs(k-self._previous_k) > limit/3:
                self._hits, self._since = 0, None
            self._since = stamp_ns if self._since is None else self._since
            self._hits += 1
            self._previous_k = k
            self.phase = 'CONFIRM'
            if not self._confirmed(self._hits, self._since, stamp_ns):
                return self._result('RECOVERY_CONFIRMING', '安全候选连续确认中，仍停车')
            self.phase = 'MOVE'
        self._previous_k = k
        self._moved = True
        # 在每个进展窗口内固定世界目标；只有位姿确实前进后才推进锚点，
        # 避免滑动中线把原地不动误判成进展，也避免走过旧锚点后误报停滞。
        if self._progress_ns is None:
            self._goal = goal_world.copy()
            self._best_distance = float(np.linalg.norm(pose-self._goal))
            self._progress_ns = stamp_ns
        actual_distance = float(np.linalg.norm(pose-self._goal))
        if actual_distance < self._best_distance-cfg.min_progress_m:
            self._goal = goal_world.copy()
            self._best_distance = float(np.linalg.norm(pose-self._goal))
            self._progress_ns = stamp_ns
        if (stamp_ns-self._progress_ns)*1e-9 > cfg.progress_timeout_s:
            return self._fault_result('恢复动作持续执行但未取得足够进展')
        return self._result('OK', '沿可信道路/出口引导执行已验证安全短弧',
                            heading_rad=math.atan(k*self.turn_config.lookahead_m/2),
                            speed_mps=cfg.speed_mps, curvature_per_m=k,
                            swept_body=tuple(map(tuple, poses)))

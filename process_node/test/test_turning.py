"""右转状态机、独立墙段场景和曲线整车包络；不把回放当作实车验证。"""

from dataclasses import replace
import math

import numpy as np
import pytest

from process_node.corridor import CorridorTracker
from process_node.exits import ExitObservation, ExitResult, ExitTracker, TrackedExit
from process_node.perception import PlannerConfig, prepare_scan
from process_node.turning import RightTurnBehavior, TurnConfig, arc_poses, check_swept_path
from test_corridor import feedback, scene
from test_exits import corridor, road, raycast
from test_perception import make_scan


def behavior(**kwargs):
    return RightTurnBehavior(TurnConfig(**kwargs), PlannerConfig())


def exit_result(identifier=7, hint=False, center=(2.3, -1.1), state=None):
    obs = ExitObservation(center, -math.pi/2 if not hint else -.6, 1.8, 1.,
                          'CORNER_HINT' if hint else 'SIDE', not hint, 15, .8)
    track = TrackedExit(identifier, obs, state or ('SUSPECTED' if hint else 'CONFIRMED'),
                        'RIGHT', 0., 3)
    return ExitResult('EXIT_TRACKING', 'test', (track,), identifier, 'RIGHT_AVAILABLE', 0.)


def lock(b):
    return b.update(road(), feedback(), 10**9, corridor(), exit_result(), 0.)


def test_right_branch_with_open_front_starts_approach_and_locks_a_world_path():
    b = behavior()
    result = lock(b)
    assert result.phase == 'APPROACH' and result.status == 'OK'
    assert result.active and result.exit_id == 7
    assert result.speed_mps == .1
    assert b._path is not None
    original = b._path.copy()
    # 即使推荐变成另一入口，也不能改选；旧出口消失不删除已提交的路径。
    result = b.update(road(), feedback(), 1100000000, None, exit_result(42, center=(3., -1.1)))
    assert result.exit_id == 7
    np.testing.assert_array_equal(b._path, original)


def test_suspected_hint_only_approaches_incoming_axis_and_cannot_commit_a_turn():
    b = behavior()
    result = b.update(road(), feedback(), 10**9, corridor(), exit_result(hint=True), 0.)
    assert result.phase == 'APPROACH' and result.heading_rad == 0.
    assert b._path is None
    for i in range(1, 5):
        result = b.update(road(), feedback(), round((1+i*.1)*1e9), None, exit_result(hint=True))
        assert result.phase == 'APPROACH' and b._path is None
    result = b.update(road(), feedback(), 1500000000, corridor(), exit_result())
    assert b._path is not None and result.exit_id == 7


def test_hint_expiry_cancels_approach_without_turning_on_an_old_opening():
    b = behavior()
    b.update(road(), feedback(), 10**9, corridor(), exit_result(hint=True), 0.)
    for i in range(1, 8):
        result = b.update(road(), feedback(), round((1+i*.1)*1e9), corridor(), ExitResult('NONE', ''))
    assert result.phase == 'CRUISE' and not result.active


def test_left_branch_does_not_trigger_a_right_turn():
    b = behavior()
    ex = exit_result()
    ex = replace(ex, tracks=(replace(ex.tracks[0], side='LEFT'),))
    assert not b.update(road(branch='left'), feedback(), 10**9, corridor(), ex).active


@pytest.mark.parametrize('state', ['HELD', 'TENTATIVE', 'UNVERIFIED'])
def test_unconfirmed_or_held_exit_cannot_start_turn(state):
    assert not behavior().update(road(), feedback(), 10**9, corridor(), exit_result(state=state)).active


def test_new_obstacle_stops_same_path_and_clear_scan_allows_resume():
    b = behavior()
    lock(b)
    scan = make_scan(count=1440)
    scan.ranges = road().ranges.copy()
    scan.ranges[0] = .65
    result = b.update(prepare_scan(scan, PlannerConfig()), feedback(), 1100000000, None, None)
    assert result.status == 'TURN_BLOCKED' and result.heading_rad is None
    assert result.exit_id == 7 and b._path is not None
    result = b.update(road(), feedback(), 1200000000, None, None)
    assert result.status == 'OK' and result.heading_rad == 0.


@pytest.mark.parametrize('stamp', [1000000000, 900000000, 1600000000])
def test_active_time_anomaly_latches_stop_without_reselecting_an_exit(stamp):
    b = behavior()
    lock(b)
    result = b.update(road(), feedback(), stamp, corridor(), exit_result())
    assert result.phase == 'FAULT' and result.heading_rad is None
    result = b.update(road(), feedback(), 1700000000, corridor(), exit_result(99))
    assert result.phase == 'FAULT' and result.exit_id == 7


def test_bad_feedback_cannot_use_a_locked_path():
    b = behavior()
    lock(b)
    result = b.update(road(), feedback(valid=False), 1100000000, None, None)
    assert result.phase == 'FAULT' and result.heading_rad is None


def test_yaw_alone_does_not_count_as_turn_completion():
    b = behavior()
    lock(b)
    result = b.update(road(pose=(0., 0., -math.pi/2)), feedback(yaw=-math.pi/2),
                      1100000000, corridor((0., 0., -math.pi/2)), None)
    assert result.phase != 'CRUISE'


@pytest.mark.parametrize('gate_bias', [0., .35])
def test_kinematic_right_turn_runs_all_four_phases_and_reacquires_new_road(gate_bias):
    # 用解析墙段和受限曲率的位姿积分验证几何闭环；不是底层 steer 响应模型。
    b, exits, walls = behavior(), ExitTracker(), CorridorTracker()
    x = y = yaw = 0.
    phases, resets = [], 0
    # 覆盖完整行为预算；超时会作为 FAULT 明确使测试失败。
    for i in range(math.ceil(b.config.max_duration_s/.1)+5):
        ns = round((1+i*.1)*1e9)
        data = road(pose=(x, y, yaw))
        motion = feedback(x, y, yaw)
        estimate = walls.update(data, motion, ns)
        tracks = exits.update(data, motion, ns, estimate)
        if gate_bias and b._path is None:
            # 可见入口偏向远侧边界；对齐必须依靠新道路中线修正，不能永远跟这个偏置。
            from process_node.turning import rotation
            delta = np.array([gate_bias, 0.]) @ rotation(yaw)
            tracks = replace(tracks, tracks=tuple(replace(t, observation=replace(
                t.observation, center_xy=tuple(np.array(t.observation.center_xy)+delta)))
                if t.side == 'RIGHT' else t for t in tracks.tracks))
        result = b.update(data, motion, ns, estimate, tracks, estimate.target_angle_rad)
        if not phases or phases[-1] != result.phase:
            phases.append(result.phase)
        assert result.status == 'OK', result.reason
        if result.reset_corridor:
            walls.reset()
            resets += 1
        if result.active:
            ds, k = .01, result.curvature_per_m
            assert abs(k) <= 1/b.config.min_radius_m+1e-9
            x += ds*math.cos(yaw+k*ds/2)
            y += ds*math.sin(yaw+k*ds/2)
            yaw += k*ds
        elif i > 10:
            break
    assert phases == ['CRUISE', 'APPROACH', 'TURN', 'EXIT_ALIGN', 'CRUISE']
    assert resets == 1
    assert y < -1.4 and abs(yaw+math.pi/2) < math.radians(15)
    assert abs(x-2.3) < .3
    assert b._passed
    # 同位置同方向的完成入口换编号后，也不能被重复当成新转弯。
    old_gate, old_heading = b._passed[0]
    local = (old_gate-[x, y]) @ np.array([[math.cos(yaw), -math.sin(yaw)],
                                       [math.sin(yaw), math.cos(yaw)]])
    obs = ExitObservation(tuple(local), old_heading-yaw, 1.8, 1., 'SIDE', True, 15, .8)
    repeated = ExitResult('EXIT_TRACKING', '', (TrackedExit(100, obs, 'CONFIRMED', 'RIGHT', 0., 3),))
    result = b.update(road(pose=(x, y, yaw)), feedback(x, y, yaw), ns+100000000,
                      estimate, repeated)
    assert not result.active


def test_turn_curve_detects_an_obstacle_that_is_outside_a_straight_sweep():
    # 障碍在圆弧内侧，直行 x 方向包络不会触及它。
    p = PlannerConfig()
    data = raycast([((-5., 3.), (5., 3.)), ((-5., -3.), (5., -3.)),
                    ((.7, -.85), (.9, -.85))])
    assert check_swept_path(data, arc_poses(0., 1.3, .04), p)[0]
    safe, reason = check_swept_path(data, arc_poses(-1., 1.3, .04), p)
    assert not safe  # 在碰撞发生前，也可能先被障碍造成的遮挡拒绝。


def test_curve_checks_rear_swing_not_just_vehicle_center():
    p = PlannerConfig(rear_overhang_m=.7)
    data = raycast([((-4., 3.), (4., 3.)), ((-4., -3.), (4., -3.)),
                    ((-.55, .45), (-.50, .45))])
    assert check_swept_path(data, arc_poses(0., .6, .02), p)[0]
    assert not check_swept_path(data, arc_poses(-1., .6, .02), p)[0]


def test_unknown_side_region_and_no_return_limit_cannot_be_crossed():
    p = PlannerConfig()
    data = prepare_scan(scene(), p)
    assert check_swept_path(data, arc_poses(0., .8, .04), p)[0]
    data.bin_ranges[np.abs(data.bin_angles+.3)<.04] = np.nan
    assert not check_swept_path(data, arc_poses(0., .8, .04), p)[0]
    limited = PlannerConfig(no_return_distance_m=.7)
    assert not check_swept_path(prepare_scan(scene(), limited), arc_poses(0., .8, .04), limited)[0]


def test_sensor_translation_is_applied_to_visibility_and_obstacles():
    p = PlannerConfig()
    sensor = (.2, .1)
    data = prepare_scan(scene(sensor=sensor), p)
    assert check_swept_path(data, arc_poses(0., .8, .04), p, sensor)[0]


@pytest.mark.parametrize('kwargs', [dict(min_radius_m=0.), dict(min_radius_m=2.),
                                      dict(radius_samples=2.5), dict(max_angle_deg=180.),
                                      dict(speed_mps=float('nan'))])
def test_bad_turn_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        TurnConfig(**kwargs)


@pytest.mark.parametrize('limit', ['duration', 'travel'])
def test_behavior_budget_stops_a_stuck_or_runaway_turn(limit):
    b = behavior(max_duration_s=.3) if limit == 'duration' else behavior(max_travel_m=.3)
    lock(b)
    for i in range(1, 6):
        f = feedback(x=.5) if limit == 'travel' else feedback()
        result = b.update(road(), f, round((1+i*.1)*1e9), None, None)
    assert result.phase == 'FAULT' and result.heading_rad is None


def test_fillet_headings_remain_continuous_across_angle_seam():
    from process_node.turning import fillet_path, rotation
    axis = math.radians(-170)
    obs = replace(exit_result().tracks[0].observation,
                  center_xy=tuple(np.array([2.3, -1.1]) @ rotation(axis).T),
                  heading_rad=math.radians(100))
    path, _, _ = fillet_path((0., 0.), axis, 0., obs, .8, TurnConfig())
    assert np.max(np.abs(np.diff(path[:, 2]))) < .06
    assert path[-1, 2] == pytest.approx(math.radians(-260))

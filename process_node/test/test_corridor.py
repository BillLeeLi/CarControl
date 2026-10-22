"""走廊几何、回中符号、历史运动补偿和失效边界的验证。"""

from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from process_node.corridor import CorridorConfig, CorridorTracker, fit_walls, select_cruise_candidate
from process_node.motion import MotionFeedback
from process_node.perception import PlannerConfig, analyze_scan, prepare_scan
from test_perception import make_scan


def scene(heading=0.0, center=0.0, width=2.2, sensor=(0.0, 0.0), count=1440):
    """由已知平行墙解析生成测距；heading/center 均在当前车体坐标中定义。"""
    scan = make_scan(count=count)
    angles = np.arange(count) * scan.angle_increment
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    normal = np.array([-math.sin(heading), math.cos(heading)])
    denominator = directions @ normal
    left = center + width / 2 - normal @ sensor
    right = center - width / 2 - normal @ sensor
    distance = np.divide(np.where(denominator > 0, left, right), denominator,
                         out=np.full(count, np.inf), where=np.abs(denominator) > 1e-8)
    scan.ranges = np.where((distance > 0) & (distance <= 4.3), distance, np.inf)
    return scan


def data(**kwargs):
    return prepare_scan(scene(**kwargs), PlannerConfig())


def feedback(x=0.0, y=0.0, yaw=0.0, valid=True):
    return MotionFeedback(NS(x_m=x, y_m=y, yaw_rad=yaw),
                          'OK' if valid else 'STALE_ODOM', 'test')


def confirm(tracker, scan_data=None, motion=None, start=1.0):
    scan_data = data() if scan_data is None else scan_data
    motion = feedback() if motion is None else motion
    for i in range(tracker.config.confirm_frames):
        result = tracker.update(scan_data, motion, round((start + i * .1) * 1e9))
    return result


@pytest.mark.parametrize('heading,center', [
    (0.0, 0.0), (0.0, -.3), (0.0, .3),
    (math.radians(15), 0), (math.radians(-20), 0),
    (math.radians(12), -.2),
])
def test_wall_direction_center_and_reference_sign(heading, center):
    tracker = CorridorTracker(CorridorConfig(filter_tau_s=0))
    result = confirm(tracker, data(heading=heading, center=center))
    assert result.status == 'CORRIDOR_TRACKING'
    assert result.width_m == pytest.approx(2.2, abs=.03)
    assert result.heading_rad == pytest.approx(heading, abs=math.radians(.5))
    assert result.lateral_error_m == pytest.approx(center, abs=.02)
    expected = heading + math.atan2(center, 1.5)
    assert result.target_angle_rad == pytest.approx(expected, abs=math.radians(1))
    if abs(heading) > math.radians(15) or abs(center) > .25:
        assert result.speed_scale == .5


def test_reference_returns_to_zero_when_pose_returns_to_center():
    tracker = CorridorTracker(CorridorConfig(filter_tau_s=0))
    initial = confirm(tracker, data(center=-.2), feedback(y=.2))
    assert initial.target_angle_rad < 0
    returned = tracker.update(data(), feedback(y=0), round(1.3e9))
    assert returned.target_angle_rad == pytest.approx(0, abs=math.radians(.5))


def test_sensor_translation_is_used_for_wall_geometry_and_world_history():
    sensor = (.3, .1)
    tracker = CorridorTracker(CorridorConfig(sensor_x_m=sensor[0], sensor_y_m=sensor[1]))
    result = confirm(tracker, data(sensor=sensor))
    assert result.lateral_error_m == pytest.approx(0, abs=.02)
    assert result.target_angle_rad == pytest.approx(0, abs=math.radians(.5))
    assert result.sensor_offset_xy == sensor


def test_hold_compensates_translation_and_rotation_including_pi_wrap():
    tracker = CorridorTracker(CorridorConfig(filter_tau_s=0))
    confirm(tracker, data(), feedback(x=100, y=50, yaw=math.radians(179)))
    empty = prepare_scan(make_scan(ranges=np.full(1440, np.nan)), PlannerConfig())
    # 沿旧中线左法向移动 0.1 m，并左转 5 度；世界中的墙线没有移动。
    n = np.array([-math.sin(math.radians(179)), math.cos(math.radians(179))])
    moved = feedback(x=100 + .1*n[0], y=50 + .1*n[1], yaw=math.radians(-176))
    result = tracker.update(empty, moved, round(1.3e9))
    assert result.status == 'CORRIDOR_HOLD'
    assert result.heading_rad == pytest.approx(math.radians(-5), abs=math.radians(.5))
    assert result.lateral_error_m == pytest.approx(-.1, abs=.02)
    assert result.target_angle_rad < math.radians(-5)
    assert result.speed_scale == .5


def test_door_recess_and_sparse_obstacle_points_do_not_replace_walls():
    scan = scene()
    angles = np.arange(len(scan.ranges)) * scan.angle_increment
    x = scan.ranges * np.cos(angles)
    doorway = (np.sin(angles) > 0) & (x > 1) & (x < 2)
    scan.ranges[doorway] = 2.0 / np.sin(angles[doorway])
    scan.ranges[[25, 300, 450, 600]] = .7
    result = confirm(CorridorTracker(), prepare_scan(scan, PlannerConfig()))
    assert result.usable
    assert abs(result.heading_rad) < math.radians(1)
    assert abs(result.lateral_error_m) < .03
    assert result.width_m == pytest.approx(2.2, abs=.05)
    # 拟合中忽略离群点不能把它们从碰撞检查中删除。
    collision = analyze_scan(scan, PlannerConfig(), extra_headings=(0.0,))
    assert not any(abs(c.angle) < 1e-9 for c in collision.candidates)


def test_single_wall_requires_prior_width_and_has_a_pair_age_deadline():
    scan = scene()
    angles = np.arange(len(scan.ranges)) * scan.angle_increment
    scan.ranges[np.sin(angles) < 0] = np.inf
    one_side = prepare_scan(scan, PlannerConfig())
    tracker = CorridorTracker()
    assert not tracker.update(one_side, feedback(), round(.9e9)).usable
    assert confirm(tracker).usable
    for t in (1.3, 1.4, 1.5, 1.6, 1.7):
        result = tracker.update(one_side, feedback(), round(t*1e9))
        assert result.status == 'CORRIDOR_SINGLE_WALL'
        assert result.width_m == pytest.approx(2.2, abs=.03)
        assert result.speed_scale == .5
    assert not tracker.update(one_side, feedback(), round(1.9e9)).usable


def test_missing_observations_expire_and_require_reconfirmation():
    tracker = CorridorTracker()
    confirm(tracker)
    missing = prepare_scan(make_scan(ranges=np.full(1440, np.nan)), PlannerConfig())
    assert tracker.update(missing, feedback(), round(1.3e9)).usable
    assert not tracker.update(missing, feedback(), round(1.6e9)).usable
    assert tracker.update(data(), feedback(), round(1.7e9)).status == 'CORRIDOR_ACQUIRING'


def test_large_unobserved_translation_expires_history():
    tracker = CorridorTracker()
    confirm(tracker)
    missing = prepare_scan(make_scan(ranges=np.full(1440, np.nan)), PlannerConfig())
    assert not tracker.update(missing, feedback(x=.3), round(1.3e9)).usable


def test_width_or_center_jump_does_not_immediately_change_reference():
    tracker = CorridorTracker()
    confirm(tracker)
    result = tracker.update(data(width=3.0, center=.4), feedback(), round(1.3e9))
    assert result.status == 'CORRIDOR_SINGLE_WALL'  # 右墙未动，可限时沿用已确认宽度。
    assert result.width_m == pytest.approx(2.2, abs=.03)
    assert abs(result.lateral_error_m) < .02


def test_nonparallel_walls_do_not_initialize_a_corridor():
    scan = scene()
    angles = np.arange(len(scan.ranges)) * scan.angle_increment
    left = np.sin(angles) > 0
    n = np.array([-math.sin(math.radians(20)), math.cos(math.radians(20))])
    denom = n[0] * np.cos(angles[left]) + n[1] * np.sin(angles[left])
    scan.ranges[left] = np.divide(1.1, denom, out=np.full(left.sum(), np.inf), where=denom > 0)
    result = confirm(CorridorTracker(), prepare_scan(scan, PlannerConfig()))
    assert not result.usable


def test_feedback_loss_and_duplicate_time_clear_reference():
    tracker = CorridorTracker()
    confirm(tracker)
    assert not tracker.update(data(), feedback(valid=False), round(1.3e9)).usable
    assert tracker.update(data(), feedback(), round(1.4e9)).status == 'CORRIDOR_ACQUIRING'
    confirm(tracker, start=2.0)
    assert tracker.update(data(), feedback(), round(2.2e9)).status == 'CORRIDOR_TIME_INVALID'
    assert tracker.update(data(), feedback(), round(2.3e9)).status == 'CORRIDOR_ACQUIRING'


def test_reference_too_large_stops_and_candidate_selector_avoids_side_opening():
    result = confirm(CorridorTracker(), data(heading=math.radians(35), center=.4))
    assert result.status == 'CORRIDOR_OUT_OF_RANGE'
    assert not result.usable
    reference = confirm(CorridorTracker())
    candidates = analyze_scan(scene(), PlannerConfig()).candidates
    near = select_cruise_candidate(candidates, reference, CorridorConfig())
    assert near.angle == pytest.approx(0)
    side_only = [c for c in candidates if abs(c.angle) > math.radians(10)]
    assert select_cruise_candidate(side_only, reference, CorridorConfig()) is None


def test_fit_is_repeatable_and_recorded_start_contains_parallel_walls():
    with np.load(Path(__file__).parent/'data'/'corridor_start_scan.npz') as saved:
        scan = NS(ranges=saved['ranges'], **{k:float(saved[k]) for k in
                  ('angle_min', 'angle_increment', 'range_min', 'range_max')})
    scan_data = prepare_scan(scan, PlannerConfig())
    assert fit_walls(scan_data, CorridorConfig()) == fit_walls(scan_data, CorridorConfig())
    result = confirm(CorridorTracker(), scan_data)
    assert result.usable
    assert abs(result.heading_rad) < math.radians(10)


@pytest.mark.parametrize('kwargs', [
    {'ransac_trials': 0}, {'ransac_trials': 2.5}, {'max_lines': 1},
    {'min_width_m': 4}, {'max_rmse_m': .1}, {'goal_distance_m': 0},
    {'filter_tau_s': -1}, {'slow_speed_scale': 1.1}, {'max_target_deg': 90},
    {'sensor_x_m': float('nan')}, {'hold_timeout_s': .8},
])
def test_bad_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        replace(CorridorConfig(), **kwargs)

"""验证会影响选方向正确性的几何和无效值边界，而不是只检查函数能运行。"""

from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from process_node.perception import PlannerConfig, analyze_scan, prepare_scan, wrap_angle


def make_scan(count=1440, ranges=None, angle_min=0.0, reverse=False):
    step = 2 * math.pi / count * (-1 if reverse else 1)
    return SimpleNamespace(
        ranges=np.full(count, 10.0) if ranges is None else np.asarray(ranges),
        angle_min=angle_min, angle_increment=step,
        range_min=0.05, range_max=50.0,
    )


@pytest.mark.parametrize('count', [360, 720, 1658, 1673])
@pytest.mark.parametrize('angle_min,reverse', [(0.0, False), (-math.pi, False), (0.0, True)])
def test_straight_direction_independent_of_count_and_scan_order(count, angle_min, reverse):
    scan = make_scan(count, angle_min=angle_min, reverse=reverse)
    result = analyze_scan(scan, PlannerConfig())
    assert result.status == 'OK'
    assert result.target.angle == pytest.approx(0)


def test_first_inf_does_not_pin_target_or_change_original_indices():
    scan = make_scan()
    scan.ranges[0] = np.inf
    result = analyze_scan(scan, PlannerConfig())
    assert result.target.angle == pytest.approx(0)
    assert result.scan.ranges.size == len(scan.ranges)
    assert result.scan.positive_inf[0]
    assert not result.scan.valid[0]
    assert result.scan.angles[540] == pytest.approx(wrap_angle(540 * scan.angle_increment))


@pytest.mark.parametrize('value', [np.inf, np.nan, 0.0, -1.0, 51.0])
def test_no_valid_measurements_cannot_produce_a_target(value):
    result = analyze_scan(make_scan(ranges=np.full(1440, value)), PlannerConfig())
    assert result.status == 'NO_VALID_RANGES'
    assert result.target is None


@pytest.mark.parametrize('value', [-np.inf, 0.01])
def test_too_close_is_a_stop_condition_even_with_far_neighbors(value):
    scan = make_scan()
    scan.ranges[0] = value
    result = analyze_scan(scan, PlannerConfig())
    assert result.status == 'TOO_CLOSE'
    assert result.target is None
    center = np.abs(result.scan.bin_angles) < math.radians(0.9)
    assert not result.scan.inferred[center].any()


def test_a_few_far_points_in_unknown_area_do_not_form_a_passage():
    scan = make_scan()
    scan.ranges[:] = np.inf
    scan.ranges[[20, 70, 1410]] = 20.0
    assert analyze_scan(scan, PlannerConfig()).target is None


def test_short_bounded_hole_is_marked_inferred_but_long_hole_stays_unknown():
    scan = make_scan()
    angles = wrap_angle(np.arange(1440) * scan.angle_increment)
    scan.ranges[np.abs(angles) < math.radians(0.9)] = np.nan
    short = analyze_scan(scan, PlannerConfig())
    assert short.scan.inferred.any()
    assert short.target.angle == pytest.approx(0)
    scan.ranges[np.abs(angles) < math.radians(15)] = np.nan
    long = analyze_scan(scan, PlannerConfig())
    center = np.abs(long.scan.bin_angles) < math.radians(10)
    assert np.isnan(long.scan.bin_ranges[center]).all()
    assert long.target is None or abs(long.target.angle) > math.radians(15)


def test_hole_between_different_surfaces_is_not_interpolated():
    scan = make_scan()
    angles = wrap_angle(np.arange(1440) * scan.angle_increment)
    scan.ranges[angles < 0] = 3.0
    scan.ranges[np.abs(angles) < math.radians(0.9)] = np.inf
    data = prepare_scan(scan, PlannerConfig())
    assert not data.inferred[np.abs(data.bin_angles) < math.radians(0.9)].any()


def test_narrow_opening_is_rejected_even_though_center_ray_is_far():
    scan = make_scan()
    angles = wrap_angle(np.arange(1440) * scan.angle_increment)
    # 1 米外的一面墙，中央只有 0.30 米宽开口，中心射线仍能测到 10 米。
    front = np.cos(angles) > 0.01
    scan.ranges[front] = 1.0 / np.cos(angles[front])
    opening = front & (np.abs(np.tan(angles)) < 0.15)
    scan.ranges[opening] = 10.0
    assert analyze_scan(scan, PlannerConfig()).target is None


def test_single_near_obstacle_is_not_hidden_by_window_average():
    scan = make_scan()
    angles = wrap_angle(np.arange(1440) * scan.angle_increment)
    scan.ranges[np.abs(angles - math.radians(10)) < math.radians(1)] = 0.8
    result = analyze_scan(scan, PlannerConfig())
    assert result.target is not None
    assert result.target.angle < 0  # 左前方近障碍，应从右侧寻找候选方向。
    assert result.front_clearance_m < PlannerConfig().lookahead_m
    assert result.target.clearance_m >= PlannerConfig().lookahead_m


def test_mirrored_openings_produce_mirrored_directions():
    scan = make_scan()
    angles = wrap_angle(np.arange(1440) * scan.angle_increment)
    targets = []
    for sign in (-1, 1):
        scan.ranges[:] = 1.0
        scan.ranges[np.abs(wrap_angle(angles - sign * math.radians(30)))
                    < math.radians(36)] = 8.0
        result = analyze_scan(scan, PlannerConfig())
        assert result.target is not None
        targets.append(result.target.angle)
    assert targets[0] < 0 < targets[1]
    assert targets[0] == pytest.approx(-targets[1], abs=math.radians(2))


def test_mounting_offset_rotates_physical_direction():
    scan = make_scan()
    angles = wrap_angle(np.arange(1440) * scan.angle_increment)
    scan.ranges[:] = 1.0
    scan.ranges[np.abs(wrap_angle(angles + math.pi / 2)) < math.radians(36)] = 8
    result = analyze_scan(scan, PlannerConfig(sensor_yaw_offset_deg=90.0))
    assert result.target.angle == pytest.approx(0, abs=math.radians(1))


def test_empty_scan_bad_metadata_and_bad_configuration():
    scan = make_scan()
    scan.ranges = []
    assert analyze_scan(scan, PlannerConfig()).target is None
    scan.angle_increment = float('nan')
    assert analyze_scan(scan, PlannerConfig()).status == 'INVALID_METADATA'
    with pytest.raises(ValueError):
        replace(PlannerConfig(), vehicle_width_m=-1)


def corridor_scan(count=1440, angle_min=0.0, reverse=False):
    """两侧墙相距 2.2 m、前方很长无回波的直走廊。"""
    scan = make_scan(count=count, angle_min=angle_min, reverse=reverse)
    angles = wrap_angle(angle_min + np.arange(count) * scan.angle_increment)
    side = np.abs(np.sin(angles))
    scan.ranges = np.divide(1.1, side, out=np.full(count, np.inf), where=side > 1e-6)
    scan.ranges[scan.ranges > 4.0] = np.inf
    return scan


@pytest.mark.parametrize('count,angle_min,reverse', [
    (360, 0.0, False), (1658, 0.0, False), (1673, -math.pi, True),
])
def test_corridor_no_return_center_is_a_straight_candidate(count, angle_min, reverse):
    scan = corridor_scan(count, angle_min, reverse)
    original = scan.ranges.copy()
    result = analyze_scan(scan, PlannerConfig())
    assert result.status == 'OK'
    assert result.target.angle == pytest.approx(0)
    assert result.target.valid_ratio == 0  # 窗口内没有有限回波，也允许合法候选。
    assert result.target.support_ratio == 1
    assert result.target.no_return_ratio == 1
    assert result.target.clearance_m <= 5.0 - PlannerConfig().radius_m
    np.testing.assert_array_equal(scan.ranges, original)
    if angle_min == 0:
        assert np.isposinf(result.scan.ranges[0])


def test_no_return_is_not_interpolation_or_a_finite_measurement():
    scan = corridor_scan()
    result = analyze_scan(scan, PlannerConfig())
    center = np.abs(result.scan.bin_angles) < math.radians(10)
    assert (result.scan.bin_ranges[center] == 5.0).all()
    assert not result.scan.inferred[center].any()
    assert not result.scan.valid[result.scan.positive_inf].any()


def test_long_nan_region_remains_unknown_even_next_to_no_return():
    scan = corridor_scan()
    angles = wrap_angle(np.arange(len(scan.ranges)) * scan.angle_increment)
    scan.ranges[np.abs(angles) < math.radians(20)] = np.nan
    result = analyze_scan(scan, PlannerConfig())
    assert not any(abs(c.angle) < math.radians(10) for c in result.candidates)
    center = np.abs(result.scan.bin_angles) < math.radians(15)
    assert np.isnan(result.scan.bin_ranges[center]).all()


def test_no_return_endpoints_do_not_interpolate_nan_holes():
    scan = corridor_scan()
    angles = wrap_angle(np.arange(len(scan.ranges)) * scan.angle_increment)
    scan.ranges[np.abs(angles) < math.radians(0.9)] = np.nan
    result = analyze_scan(scan, PlannerConfig())
    assert not any(abs(c.angle) < 1e-9 for c in result.candidates)
    assert not result.scan.inferred.any()


@pytest.mark.parametrize('obstacle', [0.7, 0.01, -np.inf])
def test_near_obstacle_inside_no_return_center_blocks_straight(obstacle):
    scan = corridor_scan()
    scan.ranges[0] = obstacle
    result = analyze_scan(scan, PlannerConfig())
    assert not any(abs(c.angle) < 1e-9 for c in result.candidates)
    if result.target is not None:
        assert abs(result.target.angle) > math.radians(10)


def test_a_single_finite_near_point_in_no_return_bin_is_not_lost():
    scan = corridor_scan(count=3600)
    scan.ranges[0] = 0.7  # 同箱大多数为 inf，仍必须保留这个真实近点。
    data = prepare_scan(scan, PlannerConfig())
    assert np.nanmin(data.bin_ranges) == pytest.approx(0.7)
    result = analyze_scan(scan, PlannerConfig())
    assert not any(abs(c.angle) < 1e-9 for c in result.candidates)


def test_no_return_distance_is_limited_by_sensor_range_and_can_be_disabled():
    scan = corridor_scan()
    scan.range_max = 1.8
    cfg = PlannerConfig(min_scan_finite_ratio=0.01)
    data = prepare_scan(scan, cfg)
    assert data.no_return_limit_m == 1.8
    assert analyze_scan(scan, cfg).target is None  # 末端圆盘超出了本帧可规划范围。
    scan = corridor_scan()
    disabled = analyze_scan(scan, PlannerConfig(no_return_distance_m=0))
    assert not any(abs(c.angle) < 1e-9 for c in disabled.candidates)


def test_no_return_planning_distance_must_cover_the_whole_end_disk():
    cfg = PlannerConfig(no_return_distance_m=1.6)
    scan = corridor_scan()
    # 前半圈没有回波，真实墙点保留在后方用于整帧门控；不依赖旧车宽阻挡侧向有限点候选。
    angles = np.arange(len(scan.ranges))*scan.angle_increment
    scan.ranges[np.cos(angles) > 0] = np.inf
    result = analyze_scan(scan, cfg)
    assert result.target is None
    assert result.status == 'NO_CLEAR_PATH'


def test_candidate_generation_does_not_require_a_far_background_gap():
    # 周围真实点距雷达 1.85 m，少于实测尺寸粗筛的约 1.882 m；仍可直行 1.5 m。
    # 去掉窗口中央的近角度点，得到可通过、而粗开口不足以容纳圆盘的场景。
    scan = make_scan()
    angles = wrap_angle(np.arange(len(scan.ranges)) * scan.angle_increment)
    scan.ranges[:] = 1.85
    scan.ranges[np.abs(angles) < math.radians(10)] = np.inf
    result = analyze_scan(scan, PlannerConfig())
    assert not result.gaps
    assert result.target is not None
    assert result.target.angle == pytest.approx(0)


def test_missing_scan_sector_is_not_treated_as_no_return():
    scan = make_scan(count=720, angle_min=math.radians(20))
    scan.ranges = scan.ranges[:640]  # 正前方缺少扫描，不能当作 +inf 射线。
    result = analyze_scan(scan, PlannerConfig())
    assert not any(abs(c.angle) < 1e-9 for c in result.candidates)


@pytest.mark.parametrize('kwargs', [
    {'no_return_distance_m': -1.0}, {'no_return_distance_m': 6.0},
    {'min_scan_finite_ratio': -0.1}, {'min_scan_finite_ratio': 1.1},
])
def test_no_return_configuration_is_validated(kwargs):
    with pytest.raises(ValueError):
        PlannerConfig(**kwargs)


def test_recorded_corridor_start_has_a_forward_candidate():
    with np.load(Path(__file__).parent / 'data' / 'corridor_start_scan.npz') as saved:
        scan = SimpleNamespace(ranges=saved['ranges'], **{
            name: float(saved[name]) for name in
            ('angle_min', 'angle_increment', 'range_min', 'range_max')
        })
    result = analyze_scan(scan, PlannerConfig())
    assert result.status == 'OK'
    assert any(abs(c.angle) < 1e-9 for c in result.candidates)
    assert abs(result.target.angle) < math.radians(6)
    assert result.target.clearance_m >= PlannerConfig().lookahead_m


def test_measured_body_is_asymmetric_about_lidar_and_contains_all_corners():
    config = PlannerConfig()
    low, high = config.footprint_bounds()
    # 2026-10-23 测量的矩形，雷达更靠近车头；不是长 42 cm 的居中矩形。
    np.testing.assert_allclose(low, [-.26, -.11])
    np.testing.assert_allclose(high, [.16, .11])
    assert high[0]-low[0] == pytest.approx(.42)
    for corner in ((-.26, -.11), (-.26, .11), (.16, -.11), (.16, .11)):
        assert math.hypot(*corner) + config.safety_margin_m <= config.radius_m + 1e-12


def test_lidar_relative_dimensions_are_translated_to_base_link():
    low, high = PlannerConfig().footprint_bounds(sensor_offset=(.20, -.10), margin_m=.10)
    np.testing.assert_allclose(low, [-.16, -.31])
    np.testing.assert_allclose(high, [.46, .11])

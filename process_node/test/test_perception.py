"""验证会影响选方向正确性的几何和无效值边界，而不是只检查函数能运行。"""

from dataclasses import replace
import math
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
    scan.ranges[np.abs(angles) < math.radians(0.9)] = np.inf
    short = analyze_scan(scan, PlannerConfig())
    assert short.scan.inferred.any()
    assert short.target.angle == pytest.approx(0)
    scan.ranges[np.abs(angles) < math.radians(15)] = np.inf
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

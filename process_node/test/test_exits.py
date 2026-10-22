"""解析墙段生成的路口：出口几何、真实证据和 odom 身份关联。"""

import math

import numpy as np
import pytest

from process_node.corridor import CorridorEstimate
from process_node.exits import ExitConfig, ExitObservation, ExitTracker
from process_node.perception import PlannerConfig, prepare_scan
from test_corridor import feedback
from test_perception import make_scan


def road(branch='right', blocked=False, opening=(1.4, 3.2), pose=(0., 0., 0.), sensor=(0., 0.)):
    """有限墙段的首个射线交点；所有坐标在固定的世界走廊中定义。"""
    lo, hi = opening
    segments = [((-6, 1.1), (6, 1.1)), ((-6, -1.1), (6, -1.1))]
    if branch is not None:
        segments[1] = ((-6, -1.1), (lo, -1.1))
        segments += [((hi, -1.1), (6, -1.1)), ((lo, -1.1), (lo, -6)),
                     ((hi, -1.1), (hi, -6))]
    if blocked:
        segments = [s for s in segments if s != ((hi, -1.1), (6, -1.1))]
        segments.append(((hi, -1.1), (hi, 1.1)))
    if branch == 'left':
        segments = [((a[0], -a[1]), (b[0], -b[1])) for a, b in segments]
    return raycast(segments, pose, sensor)


def raycast(segments, pose=(0., 0., 0.), sensor=(0., 0.)):
    """只使用真实墙段，不从被测算法反向生成测试测距。"""
    scan = make_scan(count=1440)
    angles = np.arange(1440) * scan.angle_increment + pose[2]
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    c, s = math.cos(pose[2]), math.sin(pose[2])
    origin = np.array(pose[:2]) + np.array([[c, -s], [s, c]]) @ sensor
    distances = np.full(1440, np.inf)
    def cross(a, b):
        return a[..., 0]*b[..., 1] - a[..., 1]*b[..., 0]
    for a, b in segments:
        a, b = np.array(a), np.array(b)
        edge, relative = b-a, a-origin
        den = cross(directions, edge)
        t = np.divide(cross(relative, edge), den, out=np.full(1440, np.inf), where=abs(den)>1e-9)
        u = np.divide(cross(relative, directions), den, out=np.full(1440, np.inf), where=abs(den)>1e-9)
        hit = (t > .02) & (u >= 0) & (u <= 1)
        distances = np.minimum(distances, np.where(hit, t, np.inf))
    scan.ranges = np.where(distances <= 5, distances, np.inf)
    return prepare_scan(scan, PlannerConfig())


def corridor(pose=(0., 0., 0.)):
    return CorridorEstimate('CORRIDOR_TRACKING', 'synthetic', -pose[2], -pose[1],
                            2.2, 0., (1.5, 0.))


def sequence(tracker, data=None, start=1., pose=(0., 0., 0.)):
    for i in range(3):
        result = tracker.update(road() if data is None else data,
                                feedback(*pose), round((start+i*.1)*1e9), corridor(pose))
    return result


def test_straight_corridor_has_forward_exit_without_spurious_side_openings():
    result = sequence(ExitTracker(), road(branch=None))
    assert result.recommendation == 'FORWARD_AVAILABLE'
    assert {t.side for t in result.tracks if t.state == 'CONFIRMED'} == {'FORWARD'}


def test_right_branch_is_detected_while_forward_remains_open():
    result = sequence(ExitTracker())
    sides = {t.side for t in result.tracks if t.state == 'CONFIRMED'}
    assert sides == {'FORWARD', 'RIGHT'}
    assert result.recommendation == 'RIGHT_AVAILABLE'
    right = next(t for t in result.tracks if t.side == 'RIGHT')
    assert right.observation.width_m >= 1.
    assert right.observation.center_xy[0] == pytest.approx(2.3, abs=.3)


def test_blocked_front_does_not_prevent_right_exit_or_invent_forward_continuation():
    result = sequence(ExitTracker(), road(blocked=True))
    assert result.recommendation == 'RIGHT_AVAILABLE'
    assert not any(t.side == 'FORWARD' for t in result.tracks)


def test_left_only_geometry_is_reported_without_enforcing_right_turn_everywhere():
    result = sequence(ExitTracker(), road(branch='left', blocked=True))
    assert result.recommendation == 'LEFT_AVAILABLE'


def test_narrow_doorway_is_not_a_side_road():
    result = sequence(ExitTracker(), road(opening=(1.5, 2.2)))
    assert not any(t.side == 'RIGHT' for t in result.tracks)


def triangle(pose=(0., 0., 0.), mirror=False):
    # 侧墙向外展开后结束，出口先出现在斜前方；不是标准平行墙侧向通道。
    walls = [((-6, 1.1), (5, 1.1)), ((-6, -1.5), (1., -1.5)),
             ((1., -3.5), (4.8, .2))]
    if mirror:
        walls = [((a[0], -a[1]), (b[0], -b[1])) for a, b in walls]
    return raycast(walls, pose=pose)


@pytest.mark.parametrize('mirror,side', [(False, 'RIGHT'), (True, 'LEFT')])
def test_wall_endpoint_reveals_diagonal_hint_before_standard_side_portal(mirror, side):
    scan = triangle(mirror=mirror)
    old = sequence(ExitTracker(ExitConfig(corner_enabled=False)), scan)
    assert not any(t.side == side for t in old.tracks)
    result = sequence(ExitTracker(), scan)
    hint = next(t for t in result.tracks if t.side == side)
    assert hint.state == 'SUSPECTED'
    assert hint.observation.role == 'CORNER_HINT'
    assert not hint.observation.structural
    assert result.preferred_id != hint.track_id


def test_hints_without_corridor_never_confirm_or_recommend_even_after_many_frames():
    tracker = ExitTracker()
    for i in range(10):
        result = tracker.update(triangle(), feedback(), round((1+i*.1)*1e9))
        assert result.status == 'EXIT_HINTS_ONLY'
        assert result.preferred_id is None
        assert all(t.state == 'SUSPECTED' and t.observation.role == 'CORNER_HINT'
                   for t in result.tracks)


@pytest.mark.parametrize('yaw', [math.radians(170), math.radians(-170)])
def test_hint_geometry_across_scan_seam_keeps_side_in_approach_frame(yaw):
    pose = (0., 0., yaw)
    result = sequence(ExitTracker(), triangle(pose), pose=pose)
    assert any(t.side == 'RIGHT' and t.state == 'SUSPECTED' for t in result.tracks)


def test_hint_upgrade_keeps_id_and_requires_new_standard_confirmation(monkeypatch):
    hint = ExitObservation((1., -1.5), math.radians(-40), 1.1, .6,
                           'CORNER_HINT', False, 8, .4)
    portal = ExitObservation((1.5, -1.3), math.radians(-90), 1.2, 1., 'SIDE', True, 15, .8)
    current = [hint]
    monkeypatch.setattr('process_node.exits.detect_exits', lambda *args: current)
    tracker = ExitTracker()
    before = sequence(tracker)
    identifier = before.tracks[0].track_id
    current[:] = [portal]
    for t in (1.3, 1.4):
        result = tracker.update(road(), feedback(), round(t*1e9), corridor())
        assert result.tracks[0].track_id == identifier
        assert result.tracks[0].state == 'TENTATIVE'
        assert result.preferred_id is None
    result = tracker.update(road(), feedback(), round(1.5e9), corridor())
    assert result.tracks[0].state == 'CONFIRMED'
    assert result.preferred_id == identifier


def test_hint_state_is_discarded_with_invalid_motion():
    tracker = ExitTracker()
    tracker.update(triangle(), feedback(), 10**9)
    result = tracker.update(triangle(), feedback(valid=False), round(1.1e9))
    assert result.status == 'EXIT_INPUT_INVALID'
    assert not result.tracks


def test_t_junction_with_both_sides_open_prefers_right():
    segments = [((-6, 1.1), (1.4, 1.1)), ((-6, -1.1), (1.4, -1.1)),
                ((1.4, 1.1), (1.4, 6)), ((1.4, -1.1), (1.4, -6)),
                ((3.2, -6), (3.2, 6))]
    result = sequence(ExitTracker(), raycast(segments))
    assert {t.side for t in result.tracks if t.state == 'CONFIRMED'} == {'LEFT', 'RIGHT'}
    assert result.recommendation == 'RIGHT_AVAILABLE'


@pytest.mark.parametrize('offset', [-20., 20.])
def test_slanted_corner_can_report_a_non_rectangular_side_exit(offset):
    direction = np.array([math.cos(math.radians(-90+offset)),
                          math.sin(math.radians(-90+offset))])
    a, b = np.array([1.4, -1.1]), np.array([3.2, -1.1])
    walls = [((-6, 1.1), (6, 1.1)), ((-6, -1.1), tuple(a)),
             (tuple(b), (6, -1.1)), (tuple(a), tuple(a+5*direction)),
             (tuple(b), tuple(b+5*direction))]
    # 向车后倾斜的出口要更接近入口后才可看见纵深，不能要求穿墙提前识别。
    result = sequence(ExitTracker(), raycast(walls, pose=(.6, 0., 0.)), pose=(.6, 0., 0.))
    assert result.recommendation == 'RIGHT_AVAILABLE'


def test_short_z_segment_can_report_opposite_next_exit_without_centering():
    # 两条平行直路由短连接段接起来；先左转，再右转。中间不是长直走廊。
    walls = [((-6, -1.1), (3.2, -1.1)), ((3.2, -1.1), (3.2, 1.5)),
             ((3.2, 1.5), (7, 1.5)), ((-6, 1.1), (1.4, 1.1)),
             ((1.4, 1.1), (1.4, 3.7)), ((1.4, 3.7), (7, 3.7))]
    tracker = ExitTracker()
    initial = sequence(tracker, raycast(walls, pose=(.3, 0., 0.)), pose=(.3, 0., 0.))
    assert initial.recommendation == 'LEFT_AVAILABLE'
    # 第一弯后保留 0.15 m 横向偏差，仍允许马上确认右侧出口。
    pose = (2.45, 1.0, math.pi/2)
    context = CorridorEstimate('CORRIDOR_TRACKING', 'connector', 0., .15, 1.8, 0., (1.5, 0.))
    for i in range(3):
        result = tracker.update(raycast(walls, pose=pose), feedback(*pose),
                                round((1.3+i*.1)*1e9), context)
    assert result.recommendation == 'RIGHT_AVAILABLE'


def test_independent_real_obstacle_inside_exit_prevents_confirmation():
    # 在出口中放置短墙段；可见空隙宽度或纵深不足时不得仍确认道路。
    blocked = raycast([((-6, 1.1), (6, 1.1)), ((-6, -1.1), (1.4, -1.1)),
                       ((3.2, -1.1), (6, -1.1)), ((1.4, -1.1), (1.4, -6)),
                       ((3.2, -1.1), (3.2, -6)), ((1.4, -1.5), (3.2, -1.5))])
    result = sequence(ExitTracker(), blocked)
    assert not any(t.side == 'RIGHT' and t.state == 'CONFIRMED' for t in result.tracks)


def test_inf_patch_with_no_boundary_structure_cannot_confirm_a_side_road():
    data = road()
    # 保留车后墙面满足整帧有效性，前/侧全部 inf 不能充当道路边界。
    scan = make_scan(count=1440)
    scan.ranges = np.where(np.cos(data.angles) < -.7, data.ranges, np.inf)
    result = sequence(ExitTracker(), prepare_scan(scan, PlannerConfig()))
    assert result.preferred_id is None
    assert not any(t.state == 'CONFIRMED' for t in result.tracks)


def test_unknown_region_and_unscanned_directions_cannot_become_exits():
    scan = make_scan(count=1440)
    scan.ranges = road().ranges.copy()
    angles = np.arange(1440)*scan.angle_increment
    scan.ranges[(np.sin(angles)<0) & (np.cos(angles)>0)] = np.nan
    result = sequence(ExitTracker(), prepare_scan(scan, PlannerConfig()))
    assert not any(t.side == 'RIGHT' for t in result.tracks)


def test_confirmation_requires_frames_and_elapsed_message_time():
    tracker = ExitTracker()
    for t in (1., 1.01, 1.02):
        result = tracker.update(road(), feedback(), round(t*1e9), corridor())
        assert result.preferred_id is None
    result = tracker.update(road(), feedback(), round(1.2e9), corridor())
    assert result.recommendation == 'RIGHT_AVAILABLE'


def test_world_identity_survives_translation_and_rotation_without_new_corridor():
    tracker = ExitTracker()
    initial = sequence(tracker)
    identifier = next(t.track_id for t in initial.tracks if t.side == 'RIGHT')
    pose = (.2, .1, math.radians(-10))
    result = tracker.update(road(pose=pose), feedback(*pose), round(1.3e9))
    right = next(t for t in result.tracks if t.track_id == identifier)
    assert right.state == 'CONFIRMED'
    assert right.observation.heading_rad == pytest.approx(math.radians(-80), abs=.01)


def test_sensor_translation_is_accounted_for_in_detection_and_tracking():
    sensor = (.25, .1)
    result = sequence(ExitTracker(sensor_offset=sensor), road(sensor=sensor))
    right = next(t for t in result.tracks if t.side == 'RIGHT')
    assert right.observation.center_xy == pytest.approx((2.3, -1.1), abs=.3)
    assert result.sensor_offset_xy == sensor


def test_short_occlusion_holds_id_but_does_not_recommend_old_exit():
    tracker = ExitTracker()
    initial = sequence(tracker)
    identifier = initial.preferred_id
    result = tracker.update(road(branch=None), feedback(), round(1.3e9))
    assert next(t for t in result.tracks if t.track_id == identifier).state == 'HELD'
    assert result.preferred_id != identifier
    result = tracker.update(road(), feedback(), round(1.4e9))
    right = next(t for t in result.tracks if t.track_id == identifier)
    assert right.state == 'TENTATIVE'  # 不能把丢失前的连续确认直接续上。


@pytest.mark.parametrize('t,pose', [(1.8, (0., 0., 0.)), (1.3, (.6, 0., 0.))])
def test_unseen_tracks_expire_by_time_or_distance(t, pose):
    tracker = ExitTracker()
    initial = sequence(tracker)
    identifier = initial.preferred_id
    result = tracker.update(road(branch=None, pose=pose), feedback(*pose), round(t*1e9))
    assert not any(track.track_id == identifier for track in result.tracks)


def test_feedback_failure_and_time_rewind_clear_history_and_do_not_reuse_ids():
    tracker = ExitTracker()
    first = sequence(tracker)
    maximum = max(t.track_id for t in first.tracks)
    invalid = tracker.update(road(), feedback(valid=False), round(1.3e9))
    assert invalid.status == 'EXIT_INPUT_INVALID' and not invalid.tracks
    next_result = sequence(tracker, start=2.)
    assert min(t.track_id for t in next_result.tracks) > maximum
    rewind = tracker.update(road(), feedback(), round(1.0e9))
    assert rewind.status == 'EXIT_TIME_INVALID' and not rewind.tracks


def test_context_can_survive_cruise_loss_but_expires_without_new_walls():
    tracker = ExitTracker()
    sequence(tracker)
    for t in np.arange(1.3, 4.4, .2):
        result = tracker.update(road(), feedback(), round(t*1e9))
    assert result.status == 'EXIT_NO_CONTEXT'
    assert not result.tracks


def test_no_initial_corridor_means_no_assumed_width_or_branch():
    assert ExitTracker().update(road(), feedback(), 10**9).status == 'EXIT_NO_CONTEXT'


@pytest.mark.parametrize('kwargs', [dict(min_width_m=-1), dict(sample_step_m=1.),
                                   dict(match_angle_deg=90.), dict(confirm_frames=1.5),
                                   dict(enabled=1), dict(context_timeout_s=float('nan'))])
def test_invalid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        ExitConfig(**kwargs)

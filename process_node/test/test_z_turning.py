"""左右转选择与短 Z：真实墙段射线、曲率约束、无需中间回中及故障保护。"""

from dataclasses import replace
import math

import numpy as np
import pytest

from process_node.corridor import CorridorTracker
from process_node.exits import ExitResult, ExitTracker, ExitConfig, ExitApproach, detect_corner_roads
from process_node.perception import PlannerConfig
from process_node.turning import TurnBehavior, TurnConfig, connector_paths, fillet_path, check_swept_path, arc_poses
from test_corridor import feedback
from test_exits import corridor, road, raycast
from test_turning import exit_result


def left_exit(state='CONFIRMED'):
    result = exit_result(state=state)
    t = result.tracks[0]
    obs = replace(t.observation, center_xy=(2.3, 1.1), heading_rad=math.pi/2)
    return replace(result, tracks=(replace(t, side='LEFT', observation=obs),))


def z_road(pose=(0., 0., 0.), offset=2., obstacle=False, sensor=(0., 0.)):
    # 两条平行道路由短纵向连接段连接，尺寸仅用于测试，不是控制器的路线坐标。
    walls = [((-6., -1.1), (3.2, -1.1)), ((3.2, -1.1), (3.2, offset-1.1)),
             ((3.2, offset-1.1), (8., offset-1.1)),
             ((-6., 1.1), (1.4, 1.1)), ((1.4, 1.1), (1.4, offset+1.1)),
             ((1.4, offset+1.1), (8., offset+1.1))]
    if obstacle:
        walls.append(((3.4, offset-1.1), (3.4, offset+1.1)))
    return raycast(walls, pose, sensor)


def test_left_requires_confirmed_exit_and_continuous_real_front_barrier():
    b = TurnBehavior()
    for i in range(3):
        result = b.update(road(branch='left', blocked=True), feedback(),
                          round((1+i*.1)*1e9), corridor(), left_exit())
        if i < 2:
            assert result.status == 'TURN_LEFT_CONFIRMING' and result.heading_rad is None
    assert result.side == 'LEFT' and result.phase == 'APPROACH'
    assert b._path is not None and b._path[-1, 2] > 0


@pytest.mark.parametrize('state', ['HELD', 'TENTATIVE', 'UNVERIFIED', 'SUSPECTED'])
def test_unconfirmed_left_does_not_trigger_even_when_forward_is_blocked(state):
    # 隔离普通入口确认路径；真实转角的独立证据另有专门测试。
    b = TurnBehavior(exit_config=ExitConfig(corner_road_enabled=False))
    for i in range(5):
        result = b.update(road(branch='left', blocked=True), feedback(), round((1+i*.1)*1e9),
                          corridor(), left_exit(state))
    assert not result.active


def test_corner_boundary_evidence_must_confirm_independently_before_left_control():
    b = TurnBehavior()
    for i in range(5):
        r = b.update(z_road(), feedback(), round((1+i*.1)*1e9), corridor(), ExitResult('NONE',''))
        if i<4:
            assert b._path is None
    assert r.side == 'LEFT' and r.entry_basis == 'CORNER_PAIR'
    assert r.exit_id<0 and b._path is not None


def test_unknown_space_or_continuous_side_walls_cannot_create_corner_road():
    assert not detect_corner_roads(road(branch=None), ExitConfig(), ExitApproach(0.,0.,2.2))
    # 全 inf 没有任何真实横墙/端点，即使深度上限很大也不构成道路结构。
    from test_perception import make_scan
    from process_node.perception import prepare_scan
    scan=make_scan(count=1440)
    scan.ranges[:]=np.inf
    assert not detect_corner_roads(prepare_scan(scan,PlannerConfig()),ExitConfig(),ExitApproach(0.,0.,2.2))


def test_left_door_or_open_left_branch_does_not_override_forward_cruise():
    b = TurnBehavior()
    for i in range(5):
        result = b.update(road(branch='left'), feedback(), round((1+i*.1)*1e9),
                          corridor(), left_exit(), 0.)
    assert not result.active


def test_confirmed_right_keeps_priority_when_both_sides_are_reported():
    b = TurnBehavior()
    both = replace(exit_result(), tracks=exit_result().tracks+left_exit().tracks)
    result = b.update(road(blocked=True), feedback(), 10**9, corridor(), both)
    assert result.side == 'RIGHT' and result.active


def test_left_control_can_be_disabled_without_disabling_right():
    b = TurnBehavior(TurnConfig(left_enabled=False))
    for i in range(5):
        result = b.update(road(branch='left', blocked=True), feedback(), round((1+i*.1)*1e9),
                          corridor(), left_exit())
    assert not result.active


def test_signed_fillet_is_a_geometric_mirror_for_left_and_right():
    cfg = TurnConfig()
    right = fillet_path((0., 0.), 0., 0., exit_result().tracks[0].observation, .8, cfg)[0]
    left = fillet_path((0., 0.), 0., 0., left_exit().tracks[0].observation, .8, cfg)[0]
    np.testing.assert_allclose(left, right*[1., -1., -1.], atol=1e-12)


def test_single_left_runs_to_new_corridor_without_inventing_a_second_turn():
    b, walls, exits = TurnBehavior(), CorridorTracker(), ExitTracker()
    x = y = yaw = 0.
    phases, started = [], False
    for i in range(650):
        ns = round((1+i*.1)*1e9)
        data, motion = road(branch='left', blocked=True, pose=(x,y,yaw)), feedback(x,y,yaw)
        estimate = walls.update(data, motion, ns)
        tracks = exits.update(data, motion, ns, estimate)
        result = b.update(data, motion, ns, estimate, tracks, estimate.target_angle_rad)
        assert result.status in ('OK', 'TURN_LEFT_CONFIRMING'), result.reason
        if not phases or phases[-1] != result.phase:
            phases.append(result.phase)
        if result.reset_corridor:
            walls.reset()
        if result.active and result.heading_rad is not None:
            started = True
            ds, k = .01, result.curvature_per_m
            x += ds*math.cos(yaw+k*ds/2)
            y += ds*math.sin(yaw+k*ds/2)
            yaw += k*ds
        elif not result.active:
            if started:
                break
            if estimate.usable:
                assert check_swept_path(data,arc_poses(0.,.8,.04),b.planner)[0]
                x += .01  # 正常接近，直行可见净空仍须由整车短弧检查支持。
    assert phases == ['CRUISE', 'LEFT_CONFIRM', 'APPROACH', 'TURN', 'EXIT_ALIGN', 'CRUISE']
    assert y > 1.4 and abs(yaw-math.pi/2) < math.radians(15)
    assert b._passed


@pytest.mark.parametrize('offset', [1.5, 2., 2.6])
def test_short_z_uses_confirmed_second_exit_and_returns_to_parallel_road(offset):
    b, walls, exits = TurnBehavior(), CorridorTracker(), ExitTracker()
    x = y = yaw = 0.
    sides, chained_yaw, started = [], None, False
    for i in range(920):
        ns = round((1+i*.1)*1e9)
        data, motion = z_road((x,y,yaw), offset), feedback(x,y,yaw)
        estimate = walls.update(data, motion, ns)
        tracks = exits.update(data, motion, ns, estimate)
        result = b.update(data, motion, ns, estimate, tracks, estimate.target_angle_rad)
        assert result.status in ('OK', 'TURN_LEFT_CONFIRMING'), (offset, i, result.reason)
        if b._path is not None and (not sides or sides[-1] != result.side):
            sides.append(result.side)
        if result.chain_stage == 2 and chained_yaw is None:
            chained_yaw = yaw
            assert result.chain_exit.state == 'CONFIRMED'
        if result.reset_corridor:
            walls.reset()
        if result.active and result.heading_rad is not None:
            started = started or b._path is not None
            assert check_swept_path(data, result.swept_body, b.planner)[0]
            ds, k = .01, result.curvature_per_m
            assert abs(k) <= 1/b.config.min_radius_m+1e-9
            x += ds*math.cos(yaw+k*ds/2)
            y += ds*math.sin(yaw+k*ds/2)
            yaw += k*ds
        elif not result.active:
            if started:
                break
            if estimate.usable:
                assert check_swept_path(data,arc_poses(0.,.8,.04),b.planner)[0]
                x += .01
    assert sides == ['LEFT', 'RIGHT'] and chained_yaw is not None
    assert chained_yaw < math.radians(85)  # 不要求先完成整整 90° 或回中。
    assert result.phase == 'CRUISE'
    assert x > 3.5 and abs(y-offset) < .3 and abs(yaw) < math.radians(15)
    assert len(b._passed) == 2


def test_chain_connector_obeys_curvature_bound_and_is_tangent_at_both_ends():
    paths = list(connector_paths((2.3, -.8), -.6, .7, .04))
    assert paths
    for path in paths:
        assert path[0, 2] == pytest.approx(0.)
        assert path[-1, 2] == pytest.approx(-.6)
        np.testing.assert_allclose(path[-1, :2], [2.3, -.8])
        ds = np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1)
        assert np.max(np.abs(np.diff(path[:,2])/ds)) <= 1/.7+.01


@pytest.mark.parametrize('state',['TENTATIVE','HELD','UNVERIFIED','SUSPECTED'])
def test_second_leg_cannot_commit_from_unconfirmed_or_held_exit(monkeypatch,state):
    b=TurnBehavior()
    for i in range(3):
        b.update(road(branch='left',blocked=True),feedback(),round((1+i*.1)*1e9),corridor(),left_exit())
    index=int(np.argmin(np.abs(b._path[:,2]-math.radians(50))))
    x,y,yaw=b._path[index]
    from process_node.turning import rotation
    obs=replace(exit_result().tracks[0].observation,
                center_xy=tuple((np.array([3.2,2.])-[x,y]) @ rotation(yaw)),heading_rad=-yaw)
    track=replace(exit_result().tracks[0],observation=obs,state=state)
    monkeypatch.setattr(b._chain_tracker,'update',lambda *a,**k: ExitResult('test','',(track,)))
    result=b.update(z_road((x,y,yaw)),feedback(x,y,yaw),1300000000,None,ExitResult('NONE',''))
    assert result.chain_stage==1 and result.side=='LEFT'


@pytest.mark.parametrize('kwargs', [dict(left_enabled=1), dict(chain_enabled=1),
                                     dict(front_block_width_ratio=1.1), dict(chain_prepare_angle_deg=100.),
                                     dict(left_confirm_frames=1.5)])
def test_bad_left_or_chain_parameters_are_rejected(kwargs):
    with pytest.raises(ValueError):
        TurnConfig(**kwargs)

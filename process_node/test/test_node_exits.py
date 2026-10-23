"""直路出口识别保留巡航控制；异常输入不能保留过期的出口图形。"""

import numpy as np

from process_node.visualization import draw_laser
from test_node_corridor import node, start, tick


def test_exit_diagnostics_track_forward_and_leave_control_reference_unchanged(node):
    start(node)
    assert node.status == 'OK'
    assert node.last_exits.status == 'EXIT_ACQUIRING'
    tick(node, 100.4)
    assert node.last_exits.recommendation == 'FORWARD_AVAILABLE'
    assert node.last_control_input.reference.reference_id == 'corridor'
    assert node.last_plan.target.angle == 0.


def test_disabling_exit_diagnostics_preserves_identical_control_sequence(node):
    from dataclasses import replace
    start(node)
    commands = []
    for t in (100.4, 100.5, 100.6):
        tick(node, t)
        commands.append((node.steer_value, node.speed_value, node.status))
    node.exits.config = replace(node.exits.config, enabled=False)
    # 时间回跳清除全部控制/感知历史，以相同输入重跑。
    node.motion.reset()
    node.last_stamp_ns = None
    node.controller.reset()
    node._reset_corridor()
    start(node)
    disabled_commands = []
    for t in (100.4, 100.5, 100.6):
        tick(node, t)
        disabled_commands.append((node.steer_value, node.speed_value, node.status))
    assert commands == disabled_commands
    assert node.last_exits.status == 'DISABLED'


def test_exit_markers_and_text_render_without_opening_windows(node):
    start(node)
    tick(node, 100.4)
    image = draw_laser(node.last_plan, node.config, node.status, node.steer_value,
                       node.speed_value, node.label, node.last_command, node.last_corridor,
                       node.last_exits)
    assert image.shape == (1060, 800, 3)
    assert np.any(np.all(image == [80, 220, 255], axis=2))
    assert np.any(np.all(image == [255, 235, 0], axis=2))  # 青色控制目标仍可见。


def test_watchdog_discards_exit_tracks(node):
    import time
    start(node)
    tick(node, 100.4)
    assert node.last_exits.tracks
    node.last_received = time.monotonic()-node.timeout-1
    node._tick()
    assert node.last_exits is None
    assert node.speed_value == node.stop_speed


def test_unaligned_feedback_clears_exit_diagnostics(node):
    start(node)
    tick(node, 100.4)
    msg = tick(node, 100.5)
    msg.urg.header.stamp.sec += 5
    node.laser_scan_callback(msg)
    assert node.last_exits is None  # 发布前反馈保护还会清除缓存图形。
    assert node.speed_value == node.stop_speed

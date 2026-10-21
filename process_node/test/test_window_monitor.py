"""回归：GTK 不支持的可见性查询不能被误判为用户点击关闭。"""

from types import SimpleNamespace

import cv2
import pytest

from process_node.window_monitor import WindowMonitor


@pytest.mark.parametrize('value', [-1.0, 0.0, 'error'])
def test_unconfirmed_window_does_not_trigger_exit(monkeypatch, value):
    def visible(*args):
        if value == 'error':
            raise cv2.error('visibility unsupported')
        return value
    monkeypatch.setattr(cv2, 'getWindowProperty', visible)
    monitor = WindowMonitor(('camera', 'laser'))
    for _ in range(5):
        assert not monitor.any_closed()


@pytest.mark.parametrize('closed_value', [-1.0, 0.0, 'error'])
def test_supported_backend_detects_close_after_window_was_seen(monkeypatch, closed_value):
    values = iter([1.0, closed_value])
    def visible(*args):
        value = next(values)
        if value == 'error':
            raise cv2.error('window destroyed')
        return value
    monkeypatch.setattr(cv2, 'getWindowProperty', visible)
    monitor = WindowMonitor(('camera',))
    assert not monitor.any_closed()
    assert monitor.any_closed()


def test_native_gtk_detects_close_and_allows_slow_creation(monkeypatch):
    # GTK 不支持 OpenCV 的可见性属性，但原生窗口列表可正常反映其生命周期。
    monkeypatch.setattr(cv2, 'getWindowProperty', lambda *args: -1.0)
    windows = []
    monitor = WindowMonitor(('camera', 'laser'), gtk_windows=lambda: windows)
    assert not monitor.any_closed()
    camera = SimpleNamespace(get_title=lambda: 'camera', get_visible=lambda: True)
    laser = SimpleNamespace(get_title=lambda: 'laser', get_visible=lambda: True)
    windows.append(camera)
    assert not monitor.any_closed()  # 第二个窗口还没创建，不算被关闭。
    windows.append(laser)
    assert not monitor.any_closed()
    windows.remove(camera)
    assert monitor.any_closed()


def test_native_gtk_hidden_window_is_detected_after_being_seen():
    state = {'visible': True}
    camera = SimpleNamespace(get_title=lambda: 'camera',
                             get_visible=lambda: state['visible'])
    monitor = WindowMonitor(('camera',), gtk_windows=lambda: [camera])
    assert not monitor.any_closed()
    state['visible'] = False
    assert monitor.any_closed()

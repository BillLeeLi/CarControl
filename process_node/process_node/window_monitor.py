"""识别窗口是否被关闭，兼容课程镜像中 OpenCV 4.5 的 GTK3 后端。"""

import cv2


class WindowMonitor:
    """只有先确认窗口已打开，才根据后续状态认定用户关闭了它。"""

    def __init__(self, names, gtk_windows=None):
        self.names = tuple(names)
        self.gtk_windows = gtk_windows
        self.seen = set()

    def any_closed(self):
        if self.gtk_windows is not None:
            # get_visible 是 GTK 控件自身的属性，不是 OpenCV 不支持的
            # WND_PROP_VISIBLE。最小化不会把控件的 visible 属性改成 False。
            windows = {window.get_title(): window for window in self.gtk_windows()}
            for name in self.names:
                window = windows.get(name)
                if window is not None and window.get_visible():
                    self.seen.add(name)
                elif name in self.seen:
                    return True
            return False

        # 其他后端先尝试 OpenCV 属性。启动时的 0/-1/异常只表示尚未确认；
        # 不支持该查询的后端不应因此退出，仍可按 q/Esc 或 Ctrl+C 退出。
        for name in self.names:
            try:
                visible = cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE)
            except cv2.error:
                if name in self.seen:
                    return True
                continue
            if visible >= 1:
                self.seen.add(name)
            elif name in self.seen:
                return True
        return False


def create_window_monitor(names):
    """GTK3 与 OpenCV 在同一进程中共享控件，可直接查看原生顶层窗口。"""
    gtk3 = any(line.strip().startswith('GUI:') and 'GTK3' in line.upper()
               for line in cv2.getBuildInformation().splitlines())
    if gtk3:
        try:
            import gi
            gi.require_version('Gtk', '3.0')
            from gi.repository import Gtk
            return WindowMonitor(names, gtk_windows=Gtk.Window.list_toplevels)
        except (ImportError, ValueError):
            # 缺少可选绑定时降级为 OpenCV 查询，不能误判为用户关闭。
            pass
    return WindowMonitor(names)

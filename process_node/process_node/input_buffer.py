"""接收线程与控制/GUI 主线程之间的有界缓冲，不在接收回调中做计算。"""

from collections import deque
from dataclasses import dataclass
import threading
import time


@dataclass(frozen=True)
class ReceivedInput:
    message: object  # 原始 ROS 消息；这里只保存引用，不复制大图像
    received_at: float  # 接收回调入口的单调时间，不能用处理时间替代


class InputBuffer:
    """里程计按顺序批量取出；图像/激光只保留最新一帧，避免排队播放旧控制。"""

    def __init__(self, odom_capacity=200):
        if type(odom_capacity) is not int or odom_capacity < 2:
            raise ValueError('input_odom_queue_size 必须是至少为 2 的整数')
        self.ready = threading.Event()
        self._lock = threading.Lock()
        self._odom = deque(maxlen=odom_capacity)
        self._sensor = None
        self._overflow = False
        self.skipped_scans = 0  # 被更新扫描替换的待处理帧数，不包含 DDS 自身的丢帧

    def receive_odometry(self, message, received_at=None):
        item = ReceivedInput(message, time.monotonic() if received_at is None else received_at)
        with self._lock:
            if len(self._odom) == self._odom.maxlen:
                self._overflow = True
            self._odom.append(item)
        self.ready.set()

    def receive_sensor(self, message, received_at=None):
        item = ReceivedInput(message, time.monotonic() if received_at is None else received_at)
        with self._lock:
            if self._sensor is not None:
                self.skipped_scans += 1
            self._sensor = item
        self.ready.set()

    def take(self):
        """原子取得这一批；随后新到消息进入下一批，锁内不做 ROS/GUI 调用。"""
        with self._lock:
            odom, sensor, overflow = list(self._odom), self._sensor, self._overflow
            self._odom.clear()
            self._sensor = None
            self._overflow = False
        return odom, sensor, overflow

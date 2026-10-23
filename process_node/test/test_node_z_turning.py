"""ROS 节点中左右转/Z 接入同一 PD/PI 与恢复仲裁的集成验证。"""

from dataclasses import replace
import math

import numpy as np

from test_node_corridor import node
from test_node_baseline import message
from test_motion import odom
from test_z_turning import z_road


def tick_z(node,t,pose=(0.,0.,0.)):
    msg=message()
    data=z_road(pose)
    msg.urg.angle_increment=2*math.pi/len(data.ranges)
    msg.urg.ranges=data.ranges.astype(np.float32).tolist()
    ns=round(t*1e9)
    msg.urg.header.stamp.sec,msg.urg.header.stamp.nanosec=divmod(ns,10**9)
    node.odometry_callback(odom(t,x=pose[0],y=pose[1],yaw=pose[2]))
    node.laser_scan_callback(msg)


def test_real_node_selects_left_from_corner_geometry_and_uses_existing_controller(node):
    node.odometry_callback(odom(99.9))
    for i in range(9):
        tick_z(node,100+i*.1)
    assert node.last_turn.side=='LEFT' and node.turning._path is not None
    assert node.last_turn.entry_basis=='CORNER_PAIR'
    assert node.last_control_input.reference.reference_id.startswith('left_turn:')
    assert node.last_control_input.reference.target_speed_mps==.1
    assert not node.recovery.active
    assert node.status=='OK'


def test_real_node_z_switches_direction_without_switching_reference_mode(node):
    node.odometry_callback(odom(99.9))
    x=y=yaw=0.
    sides=[]
    for i in range(800):
        tick_z(node,100+i*.1,(x,y,yaw))
        turn=node.last_turn
        assert node.status not in ('TURN_FAULT','RECOVERY_FAULT'), node.last_command.reason
        if node.turning._path is not None and (not sides or sides[-1]!=turn.side):
            sides.append(turn.side)
        if turn.reset_corridor:
            assert node.reference_mode=='corridor'
        if sides and node.turning.phase=='CRUISE':
            break
        # 理想曲率积分仅验证集成链路；不模拟底层 steer 响应。
        if node.status=='OK' and turn.active and turn.heading_rad is not None:
            ds,k=.01,turn.curvature_per_m
            x+=ds*math.cos(yaw+k*ds/2)
            y+=ds*math.sin(yaw+k*ds/2)
            yaw+=k*ds
    assert sides==['LEFT','RIGHT']
    assert node.turning.phase=='CRUISE'
    assert x>3.5 and abs(y-2.)<.3
    tick_z(node,180.,(x,y,yaw))  # 不跨长时间继续旧控制；仍保留现有反馈时间保护。
    assert node.speed_value==node.stop_speed


def test_disabling_left_and_chain_keeps_right_only_configuration(node):
    node.turning.config=replace(node.turning.config,left_enabled=False,chain_enabled=False)
    node.odometry_callback(odom(99.9))
    for i in range(8):
        tick_z(node,100+i*.1)
    assert node.turning.phase=='CRUISE'
    assert node.last_control_input.reference.reference_id=='corridor'

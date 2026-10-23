"""有界读取独立 /scan 与 /odom，保存出口诊断 CSV/图片，不创建节点或发布控制。"""

import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import sqlite3
import time

import cv2
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

from process_node.corridor import CorridorConfig, CorridorTracker, select_cruise_candidate
from process_node.exits import ExitConfig, ExitTracker
from process_node.motion import MotionConfig, MotionEstimator
from process_node.perception import PlannerConfig, analyze_prepared_scan, prepare_scan
from process_node.visualization import draw_laser
from process_node.turning import RightTurnBehavior, TurnResult
from process_node.recovery import LocalRecoveryBehavior, RecoveryConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('database', type=Path, help='一个 rosbag SQLite .db3 文件')
    parser.add_argument('--offset', type=float, default=0., help='相对 bag 起点的秒数')
    parser.add_argument('--duration', type=float, default=30., help='分析时长，最多 120 秒')
    parser.add_argument('--scan-stride', type=int, default=1, help='每 N 帧扫描处理一帧，里程计不跳过')
    parser.add_argument('--output', type=Path, required=True, help='仓库外的结果目录')
    parser.add_argument('--turns', action='store_true', help='同时运行右转几何行为；不运行 PID、不发布控制')
    parser.add_argument('--recovery', action='store_true', help='同时运行右转与局部恢复几何行为，不运行 PID')
    args = parser.parse_args()
    args.turns = args.turns or args.recovery
    if not 0 < args.duration <= 120 or args.offset < 0 or args.scan_stride < 1:
        parser.error('duration 应在 (0, 120] 秒内，offset 应非负，scan-stride 应为正整数')
    args.output.mkdir(parents=True, exist_ok=True)
    connection = sqlite3.connect(args.database.resolve().as_uri() + '?mode=ro', uri=True)
    try:
        topics = {name: (identifier, kind) for identifier, name, kind
                  in connection.execute('SELECT id,name,type FROM topics')}
        scan_id, scan_type = topics['/scan']
        odom_id, odom_type = topics['/encoder_imu_node/odom']
        if scan_type != 'sensor_msgs/msg/LaserScan' or odom_type != 'nav_msgs/msg/Odometry':
            raise ValueError('独立雷达或里程计的话题类型不匹配')
        start = connection.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0]
        lower, upper = start+round(args.offset*1e9), start+round((args.offset+args.duration)*1e9)
        # 只取两种小消息的有界时间段；不读取占据主要体积的 AllSensor 图像载荷。
        rows = connection.execute(
            'SELECT timestamp,topic_id,data FROM messages WHERE timestamp>=? AND timestamp<? '
            'AND topic_id IN (?,?) ORDER BY timestamp LIMIT 20000',
            (lower, upper, scan_id, odom_id))
        planner = PlannerConfig()
        corridor = CorridorTracker(CorridorConfig())
        tracker = ExitTracker(ExitConfig())
        motion = MotionEstimator(MotionConfig())
        turning = RightTurnBehavior(planner=planner)
        recovery = LocalRecoveryBehavior(RecoveryConfig(), planner, turning.config) if args.recovery else None
        recovery_states = Counter()
        turn_states = Counter()
        states, recommendations = Counter(), Counter()
        count, payload, last_picture = 0, 0, -1e9
        previous_signature = None
        previous_stamp = None
        seen_scans = 0
        costs = []
        with (args.output/'frames.csv').open('w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['bag_s', 'odom', 'corridor', 'exits', 'suggestion', 'preferred_id', 'tracks',
                             'scan_dt_s', 'fitted_walls', 'corridor_reason', 'yaw_deg',
                             'behavior', 'turn_status', 'locked_exit', 'curvature', 'turn_reason',
                             'recovery_phase', 'recovery_status', 'recovery_reason', 'recovery_cause'])
            for timestamp, topic, blob in rows:
                payload += len(blob)
                received = (timestamp-start)*1e-9
                if topic == odom_id:
                    generation = motion.generation
                    motion.update(deserialize_message(blob, Odometry), received_at=received)
                    if motion.generation != generation:
                        corridor.reset()
                        tracker.reset()
                        turning.invalidate('独立里程计重建')
                        if recovery is not None:
                            recovery.interrupt('独立里程计重建', hard=True)
                    continue
                seen_scans += 1
                if (seen_scans-1) % args.scan_stride:
                    continue
                scan = deserialize_message(blob, LaserScan)
                stamp = scan.header.stamp.sec*10**9+scan.header.stamp.nanosec
                feedback = motion.feedback(stamp, now=received)
                before = time.perf_counter()
                data = prepare_scan(scan, planner)
                estimate = corridor.update(data, feedback, stamp)
                result = tracker.update(data, feedback, stamp, estimate)
                if recovery is not None and recovery.active and turning.phase == 'CRUISE':
                    turn = TurnResult('CRUISE', 'OK', '局部恢复期间暂停新转弯选择')
                else:
                    turn = turning.update(data, feedback, stamp, estimate, result,
                                          estimate.target_angle_rad if estimate.usable else None) if args.turns else None
                if turn is not None:
                    turn_states[turn.phase+':'+turn.status] += 1
                    if turn.reset_corridor:
                        corridor.reset()
                recovered = None
                if recovery is not None:
                    # 完整执行巡航候选门控，不能把未经安全检查的中线角当成可交接目标。
                    plan = analyze_prepared_scan(data, planner, extra_headings=(
                        (estimate.target_angle_rad,) if estimate.usable else ()))
                    cruise = select_cruise_candidate(plan.candidates, estimate, corridor.config) \
                        if estimate.usable and plan.status == 'OK' else None
                    status = plan.status if plan.status != 'OK' else (
                        'OK' if cruise is not None else 'CRUISE_BLOCKED' if estimate.usable else estimate.status)
                    recovered = recovery.update(data, feedback, stamp, estimate,
                                                None if cruise is None else cruise.angle,
                                                status, turn, turning.recovery_goal, received_at=received)
                    recovery_states[recovered.phase+':'+recovered.status] += 1
                    if recovered.handoff:
                        turning.finish_recovery()
                    elif recovered.phase == 'MOVE' and turn.active:
                        turning.invalidate('局部恢复接管', recoverable=True)
                costs.append((time.perf_counter()-before)*1000)
                descriptions = [dict(id=t.track_id, side=t.side, state=t.state,
                                     kind=t.observation.role,
                                     heading_deg=round(float(np.degrees(t.observation.heading_rad)), 2),
                                     x=round(t.observation.center_xy[0], 3),
                                     y=round(t.observation.center_xy[1], 3),
                                     width=round(t.observation.width_m, 3),
                                     depth_visible=round(t.observation.depth_visible_ratio, 3))
                                for t in result.tracks]
                writer.writerow([received, feedback.status, estimate.status, result.status,
                                 result.recommendation, result.preferred_id,
                                 json.dumps(descriptions),
                                 '' if previous_stamp is None else (stamp-previous_stamp)*1e-9,
                                 len(estimate.observed_walls), estimate.reason,
                                 '' if feedback.state is None else np.degrees(feedback.state.yaw_rad),
                                 '' if turn is None else turn.phase, '' if turn is None else turn.status,
                                 '' if turn is None else turn.exit_id,
                                 '' if turn is None else turn.curvature_per_m,
                                 '' if turn is None else turn.reason,
                                 '' if recovered is None else recovered.phase,
                                 '' if recovered is None else recovered.status,
                                 '' if recovered is None else recovered.reason,
                                 '' if recovered is None else recovered.cause])
                previous_stamp = stamp
                count += 1
                states[result.status] += 1
                recommendations[result.recommendation] += 1
                signature = (None if turn is None else (turn.phase, turn.status),
                             None if recovered is None else (recovered.phase, recovered.status),
                             tuple((t.side, t.state) for t in result.tracks))
                if received-last_picture >= 5 or (signature != previous_signature
                                                  and received-last_picture >= 1):
                    # 不显示成控制获准：该工具根本没有运行控制器。
                    plan = analyze_prepared_scan(data, planner)
                    picture = draw_laser(plan, planner, 'DIAGNOSTIC_ONLY', 0, 0,
                                         'exit replay', corridor=estimate, exits=result, turn=turn, recovery=recovered)
                    cv2.imwrite(str(args.output/f'exits_{received:.2f}s.png'), picture)
                    last_picture = received
                previous_signature = signature
        summary = dict(scans=count, scan_messages=seen_scans, payload_bytes=payload, status=dict(states),
                       suggestions=dict(recommendations), turns=dict(turn_states),
                       recovery=dict(recovery_states),
                       perception_ms_median=float(np.median(costs)) if costs else None,
                       perception_ms_p95=float(np.percentile(costs, 95)) if costs else None,
                       note='独立 /scan，默认配置，未运行控制器；不能等同 AllSensor 实时控制或实车结果')
        (args.output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    finally:
        connection.close()


if __name__ == '__main__':
    main()

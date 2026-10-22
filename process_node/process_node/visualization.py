"""只绘图、不创建窗口，便于在无桌面环境保存诊断图片。"""

import math

import cv2
import numpy as np


def _text(image, text, row, color=(225, 225, 225)):
    # OpenCV 默认字体不支持中文；中文解释保留在注释、终端日志和 README 中。
    cv2.putText(image, text, (15, row), cv2.FONT_HERSHEY_SIMPLEX,
                0.52, color, 1, cv2.LINE_AA)


def draw_steering(image, steer, center, limit, label, status, target_angle):
    """绘制控制量示意盘；指针角度不等于目标航向或真实车轮转角。"""
    origin = (100, 110)
    radius = 65
    offset = float(np.clip((steer - center) / limit, -1, 1)) * math.pi / 2
    tip = (round(origin[0] + radius * math.sin(offset)),
           round(origin[1] - radius * math.cos(offset)))
    cv2.circle(image, origin, radius, (0, 220, 0), 2)
    cv2.line(image, origin, tip, (255, 120, 0), 3)
    _text(image, f'{label} steer={steer}', 25)
    target = '--' if target_angle is None else f'{math.degrees(target_angle):+.1f} deg'
    _text(image, f'target={target}  {status}', 205)


def draw_laser(result, config, status, steer, speed, label, command=None, corridor=None, exits=None):
    """鸟瞰图：车头向上、车体左方在屏幕左方，与翻转后的 RGB 独立。"""
    image = np.full((1060 if exits is not None else 900, 800, 3), 23, dtype=np.uint8)
    origin = np.array([400.0, 500.0])
    scale = 240.0 / config.max_distance_m

    def pixel(x, y):
        return tuple(np.rint(origin + [-y * scale, -x * scale]).astype(int))

    def ray(angle, distance):
        return pixel(distance * math.cos(angle), distance * math.sin(angle))

    for distance in np.arange(1, config.max_distance_m + 0.01):
        cv2.circle(image, pixel(0, 0), round(distance * scale), (55, 55, 55), 1)
        _text(image, f'{distance:g}m', round(origin[1] - distance * scale))
    cv2.line(image, pixel(-config.max_distance_m, 0),
             pixel(config.max_distance_m, 0), (70, 70, 70), 1)
    cv2.line(image, pixel(0, -config.max_distance_m),
             pixel(0, config.max_distance_m), (70, 70, 70), 1)
    _text(image, f'{label} | {status} | steer={steer} speed={speed}', 24)
    _text(image, 'FRONT = +x / 0 deg, LEFT = +y / positive angle', 48)

    if result is not None:
        data = result.scan
        _text(image, f'valid={data.valid_ratio:.1%}  +inf={int(data.positive_inf.sum())}'
              f'  other invalid={int((~data.valid & ~data.positive_inf).sum())}'
              f'  total={data.ranges.size}', 72)
        front = '-- (unknown)' if result.front_clearance_m is None \
            else f'{result.front_clearance_m:.2f}m'
        _text(image, f'front planning clearance={front} | gaps={len(result.gaps)}'
              f' candidates={len(result.candidates)}', 96)

        for gap in result.gaps:
            angles = np.linspace(gap.start, gap.end, 50)
            arc = np.array([ray(a, config.lookahead_m + config.radius_m)
                            for a in angles], dtype=np.int32)
            cv2.polylines(image, [arc], False, (40, 130, 40), 3)
        # 无效/未采用的射线为红色；+inf 用蓝色显示其规划上限，不画成真实白色障碍。
        no_return = data.positive_inf & data.supported
        for angle in data.angles[~data.valid & ~no_return]:
            cv2.circle(image, ray(angle, config.max_distance_m), 2, (70, 70, 210), -1)
        for angle in data.angles[no_return]:
            cv2.circle(image, ray(angle, data.no_return_limit_m), 2, (230, 130, 60), -1)
        for angle, distance in zip(data.angles[data.valid], data.ranges[data.valid]):
            cv2.circle(image, ray(angle, min(distance, config.max_distance_m)),
                       2, (225, 225, 225), -1)
        for angle, distance in zip(data.bin_angles[data.inferred],
                                   data.bin_ranges[data.inferred]):
            cv2.circle(image, ray(angle, min(distance, config.max_distance_m)),
                       3, (0, 160, 255), 1)

        if result.target is not None and status == 'OK':
            target = result.target
            end = ray(target.angle, config.lookahead_m)
            # 绿色圆盘及边线是被检查的直线扫掠包络，不是可执行的转弯轨迹。
            normal = np.array([-math.sin(target.angle), math.cos(target.angle)])
            forward = np.array([math.cos(target.angle), math.sin(target.angle)])
            for sign in (-1, 1):
                a = sign * config.radius_m * normal
                b = a + config.lookahead_m * forward
                cv2.line(image, pixel(*a), pixel(*b), (0, 150, 0), 2)
            cv2.circle(image, end, round(config.radius_m * scale), (0, 150, 0), 2)
            _text(image, f'target={math.degrees(target.angle):+.1f}deg'
                  f' clearance={target.clearance_m:.2f}m'
                  f' support={target.support_ratio:.0%}', 120, (255, 235, 0))
            _text(image, f'finite={target.valid_ratio:.0%} no-return={target.no_return_ratio:.0%}'
                  f' mean(plan)={target.mean_distance_m:.2f}m'
                  f' inferred={target.inferred_ratio:.0%}', 144)
        elif status != 'OK':
            _text(image, 'No active target; STOP output (see status / terminal)', 120, (70, 70, 240))

    if command is not None and command.diagnostics is not None:
        diag = command.diagnostics
        _text(image, f'v_ref={diag.target_speed_mps:.3f} / requested={diag.requested_speed_mps:.3f}'
              f' / measured={diag.measured_speed_mps:+.3f} m/s', 168)
        _text(image, f'speed P/I={diag.speed_p:+.1f}/{diag.speed_i:+.1f}'
              f' steer P/D={diag.steer_p:+.1f}/{diag.steer_d:+.1f}', 192)

    if corridor is not None:
        age = '--' if corridor.measurement_age_s is None else f'{corridor.measurement_age_s:.2f}s'
        _text(image, f'{corridor.status} confidence={corridor.confidence:.2f} age={age}', 216)
        if corridor.heading_rad is not None:
            heading = corridor.heading_rad
            normal = np.array([-math.sin(heading), math.cos(heading)])
            tangent = np.array([math.cos(heading), math.sin(heading)])
            # 模型在车体原点定义；画到以雷达原点为中心的图上时减去安装平移。
            # 传入的估计记录安装偏移，避免把雷达和车体原点默认为同一点。
            sensor_offset = np.array(corridor.sensor_offset_xy)
            center = corridor.lateral_error_m
            for offset, color in ((center + corridor.width_m / 2, (180, 110, 90)),
                                  (center - corridor.width_m / 2, (180, 110, 90)),
                                  (center, (210, 110, 230))):
                a = -0.3 * tangent + offset * normal - sensor_offset
                b = 4.0 * tangent + offset * normal - sensor_offset
                cv2.line(image, pixel(*a), pixel(*b), color, 2)
            if corridor.target_point_xy is not None:
                goal = np.array(corridor.target_point_xy) - sensor_offset
                cv2.arrowedLine(image, pixel(*(-sensor_offset)), pixel(*goal), (210, 110, 230), 2)
            reference = '--' if corridor.target_angle_rad is None \
                else f'{math.degrees(corridor.target_angle_rad):+.1f}'
            _text(image, f'axis={math.degrees(heading):+.1f}deg center_left={center:+.2f}m'
                  f' width={corridor.width_m:.2f}m ref={reference}deg', 240)

    if exits is not None:
        for track in exits.tracks:
            obs = track.observation
            center = np.array(obs.center_xy) - np.array(exits.sensor_offset_xy)
            direction = np.array([math.cos(obs.heading_rad), math.sin(obs.heading_rad)])
            normal = np.array([-direction[1], direction[0]])
            color = (80, 220, 255) if track.state == 'CONFIRMED' else (110, 135, 155)
            if track.state == 'HELD':
                color = (100, 100, 100)
            cv2.line(image, pixel(*(center-normal*obs.width_m/2)),
                     pixel(*(center+normal*obs.width_m/2)), color, 2)
            cv2.arrowedLine(image, pixel(*center), pixel(*(center+direction*.6)), color, 2)
            if track.track_id == exits.preferred_id:
                cv2.circle(image, pixel(*center), 8, (240, 240, 240), 2)
            cv2.putText(image, f'E{track.track_id} {track.side[0]} {track.state[0]}',
                        pixel(*(center + .15*normal)), cv2.FONT_HERSHEY_SIMPLEX,
                        .42, color, 1, cv2.LINE_AA)
        _text(image, f'exits={exits.status}  suggested={exits.preferred_id}'
              f' {exits.recommendation}', 914, (80, 220, 255))
        _text(image, 'yellow exits: diagnostic only; white ring: route suggestion, not control', 936)
        for row, track in enumerate(exits.tracks[:4]):
            obs = track.observation
            _text(image, f'E{track.track_id} {track.side} {track.state}'
                  f' w={obs.width_m:.1f}m d={obs.depth_m:.1f}m'
                  f' deep={obs.depth_visible_ratio:.0%}'
                  f' walls={obs.wall_points} age={track.age_s:.2f}s', 962+row*24)

    # 活动目标最后画，避免参考或出口诊断覆盖青色箭头。
    if result is not None and result.target is not None and status == 'OK':
        cv2.arrowedLine(image, pixel(0, 0), ray(result.target.angle, config.lookahead_m),
                       (255, 235, 0), 3)

    cv2.circle(image, pixel(0, 0), round(config.radius_m * scale), (90, 150, 90), 1)
    rectangle = np.array([
        pixel(config.front_overhang_m, config.vehicle_width_m / 2),
        pixel(config.front_overhang_m, -config.vehicle_width_m / 2),
        pixel(-config.rear_overhang_m, -config.vehicle_width_m / 2),
        pixel(-config.rear_overhang_m, config.vehicle_width_m / 2),
    ], dtype=np.int32)
    cv2.polylines(image, [rectangle], True, (0, 220, 0), 2)
    _text(image, 'white: measured | blue: bounded no-return | red: invalid / unknown', 816)
    _text(image, 'orange: inferred hole | green: gap / straight swept footprint', 838)
    _text(image, 'brown walls / purple center & reference: model, not measured obstacles', 860)
    _text(image, 'cyan: active target | q / Esc / close window: stop and exit', 882)
    return image

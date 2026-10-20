import math

import numpy as np
import cv2
import copy
from cv_bridge import CvBridge
from rclpy.node import Node
from std_msgs.msg import Header

from robotsdk_msgs.msg import AllSensor, Control
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import rclpy
import sys
import argparse

class LaserWindowAnalyzer(Node):
    def __init__(self, window_size):
        super().__init__('laser_window_analyzer')
        self.window_size = window_size
        self.sensorTopic = "/sync_store_node/allsensor"
        self.declare_parameter('sensorTopic', self.sensorTopic)
        self.sensorTopic = self.get_parameter('sensorTopic').get_parameter_value().string_value
        self.detectOverlays = "box,labels,conf"
        #################################################
        #############yzy:subscription setting############
        #################################################
        self.subscription = self.create_subscription(
            AllSensor,
            self.sensorTopic,
            self.laser_scan_callback,
            10)
        self.subscription  # prevent unused variable warning
        # 初始化 CvBridge
        self.bridge = CvBridge()
        
        self.steer_value = 0
        #################################################
        #############yzy:publish setting#################
        #################################################
        self.publisher_ = self.create_publisher(Control, 'control', 1000)

        #################################################
        #############yzy:data initialazition#############
        #################################################
        # self.data_laser = None # LaserScan()
        # self.data_imuencoder = None # ImuEncoderAngle()
        # self.data_depth = None # Image()
        # self.data_RGBcsi = None # Image()
        # self.data_RGBxtion = None #Image()

    def draw_steering_wheel(self, image):
        # 将转盘放在左上角
        center = (100, 100)  # 左上角 (x, y)
        radius = 80
        wheel = image.copy()

        # 绘制圆形转盘
        cv2.circle(wheel, center, radius, (0, 255, 0), 2)

        # 绘制指示方向的线
        angle = self.steer_value * 90 / 600.0  # 将 steer 映射到 -90 到 90 度
        end_point = (int(center[0] + radius * np.cos(np.radians(90 - angle))),
                     int(center[1] - radius * np.sin(np.radians(90 - angle))))
        cv2.line(wheel, center, end_point, (255, 0, 0), 2)

        # 添加文字显示 steer 值
        text = f"steer: {self.steer_value:.2f}"
        cv2.putText(wheel, text, (center[0] - 60, center[1] - radius-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        # 将转盘叠加到图像上
        alpha = 0.7
        cv2.addWeighted(wheel, alpha, image, 1 - alpha, 0, image)


    #################################################
    #############yzy:different call back#############
    #################################################
    def laser_scan_callback(self, msg):
        # ranges = np.array(msg.laser_scan.ranges)
        steer, speed = self.process_core(msg.urg)
        cv_image = self.bridge.imgmsg_to_cv2(msg.xtion_color, desired_encoding='bgr8')
        cv_image = cv2.flip(cv_image,1)
        msg = Control()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # 设置当前时间为时间戳
        msg.header.frame_id = "None"  # 记录传感器是在哪个坐标系下采集的
        msg.speed = speed  # 不需要显示的类型转换即可保证正确性
        msg.steer = steer
        self.publisher_.publish(msg)
        
        self.steer_value = msg.steer
        # 在图像上绘制转盘
        self.draw_steering_wheel(cv_image)
        # 显示图像
        cv2.imshow("Steering Visualization", cv_image)
        cv2.waitKey(1)  # 每次刷新图像窗口

    #################################################
    #############yzy:process func####################
    #################################################
    def find_max_average_window(self, ranges):
        num_ranges = len(ranges)
        max_average = 0
        max_index = 0
        # Loop to find the window with the maximum average distance
        for i in range(num_ranges - self.window_size + 1):
            current_window = ranges[i:i + self.window_size]
            current_average = np.mean(current_window)
            if current_average > max_average:
                max_average = current_average
                max_index = i + self.window_size // 2
        return max_average, max_index
    def calculate_center_angle(self, index, angle_min, angle_increment):
        # Calculate the angle of the center laser of the window
        return angle_min + index * angle_increment
    #################################################
    #############yzy: final process##################
    #################################################
    def process_core(self, urg):
        P = -5
        Min_angle = 0
        Angle_increment = 1  # zht: 360 threads, 180
        Middle_angle = 540
        max_average, max_index = self.find_max_average_window(urg.ranges)
        center_angle = self.calculate_center_angle(max_index, Min_angle, Angle_increment)
        self.get_logger().info(f'Center angle: {center_angle}, total threads {len(urg.ranges)}')
        # for i in range(len(urg.ranges)):
        #     self.get_logger().info(f'see index {i}, range value{urg.ranges[i]}')
        self.get_logger().info(f'Max average distance: {max_average:.2f} at angle: {center_angle:.2f}')
        steer = int(P * (center_angle - Middle_angle))
        steer = max(min(steer, 600), -600)
        self.get_logger().info(f'steer of control is {steer}')
        speed = -100
        return steer, speed


def main(args=None):
    rclpy.init(args=args)
    window_size = 10  # Set the window size as needed
    laser_window_analyzer = LaserWindowAnalyzer(window_size)
    rclpy.spin(laser_window_analyzer)
    laser_window_analyzer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

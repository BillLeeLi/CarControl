import math

import numpy as np
import cv2
import copy
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
from skimage import io
from skimage.transform import resize
import torch
from torchvision.transforms import Normalize
from cv_bridge import CvBridge
from rclpy.node import Node
from std_msgs.msg import Header

from robotsdk_msgs.msg import AllSensor, Control

from geometry_msgs.msg import Twist

import rclpy
import sys
import argparse


from jetson_inference import detectNet, imageNet
from jetson_utils import videoSource, videoOutput, Log, cudaImage, cudaAllocMapped, cudaDeviceSynchronize, \
    cudaConvertColor, cudaFromNumpy


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
        self.nav2sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.nav2_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        # using jetson-inference
        # self.mynet = detectNet("ssd-mobilenet-v2")
        # self.mynet = imageNet('googlenet')
        # using tranferred net
        # f = open("resnet_engine_pytorch.trt", "rb")
        # self.runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
        # self.engine = self.runtime.deserialize_cuda_engine(f.read())
        # self.context = self.engine.create_execution_context()
        # self.BATCH_SIZE = 64
        # self.USE_FP16 = False
        # self.target_dtype = np.float16 if self.USE_FP16 else np.float32
        #
        # self.bridge = CvBridge()
        # self.output = videoOutput("/home/poss/tcposs.mp4")
        # self.output = videoOutput()

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
        

    def nav2_callback(self, msg):
        L = 0.2


#           process reverse
        if msg.angular.z == 1 and msg.linear.x ==0 and msg.linear.y == 0 and msg.linear.z == 0:
            speed = 100
            steer = 200
            msg = Control()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()  # 设置当前时间为时间戳
            msg.header.frame_id = "None"  # 记录传感器是在哪个坐标系下采集的
            msg.speed = int(speed)  # 不需要显示的类型转换即可保证正确性
            msg.steer = int(steer)
            self.publisher_.publish(msg)
            return

        if msg.angular.z < 1e-5:
            steer = 320
        else:
            R = msg.linear.x / msg.angular.z
            theta = math.atan(L / R) if R > 0 else 0
            theta_deg = math.degrees(theta)
            steer = theta_deg / 90 * (-600) - 500
            steer = max(min(steer, 600), -600)

        if msg.angular.z < 0:
            steer = 600
        p = -550
        steer = p*msg.angular.z + 200

        speed = msg.linear.x * (-1000)
        msg = Control()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # 设置当前时间为时间戳
        msg.header.frame_id = "None"  # 记录传感器是在哪个坐标系下采集的
        msg.speed = int(speed)  # 不需要显示的类型转换即可保证正确性
        msg.steer = int(steer)
        self.publisher_.publish(msg)

    #################################################
    #############yzy:different call back#############
    #################################################
    def laser_scan_callback(self, msg):
        # ranges = np.array(msg.laser_scan.ranges)
        steer, speed = self.process_core(msg.urg)
        # # csi color
        # # convert to cv
        # cv_img = self.bridge.imgmsg_to_cv2(msg.csi_color, desired_encoding="bgr8")
        # cv_img = cv2.flip(cv_img, 0)
        #
        # # using transferred net
        # img = resize(io.img_as_float(cv2.cvtColor(cv_img,cv2.COLOR_BGR2RGB)), (224, 224))
        # input_batch = np.array(np.repeat(np.expand_dims(np.array(img, dtype=np.float32), axis=0), self.BATCH_SIZE, axis=0),
        #                        dtype=np.float32)
        #
        # def preprocess_image(img):
        #     norm = Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        #     result = norm(torch.from_numpy(img).transpose(0, 2).transpose(1, 2))
        #     return np.array(result, dtype=np.float32)

        # preprocessed_images = np.array([preprocess_image(image) for image in input_batch])
        # # async
        # tensor_name = self.engine.get_tensor_name(0)  # input tensor
        # self.context.set_input_shape(tensor_name, [self.BATCH_SIZE, 3, 224, 224])  # use your input_shape
        # assert self.context.all_binding_shapes_specified
        # # allocate device memory
        # d_input = cuda.mem_alloc(1 * input_batch.nbytes)
        # output = np.empty([self.BATCH_SIZE, 10], dtype=self.target_dtype)
        # d_output = cuda.mem_alloc(1 * output.nbytes)
        # self.context.set_tensor_address(self.engine.get_tensor_name(0), int(d_input))  # input buffer
        # self.context.set_tensor_address(self.engine.get_tensor_name(1), int(d_output))  # output buffer
        # stream = cuda.Stream()

        # def predict(batch):  # result gets copied into output
        #     # transfer input data to device
        #     cuda.memcpy_htod_async(d_input, batch, stream)
        #     # execute model
        #     self.context.execute_async_v3(stream_handle=stream.handle)
        #     # cuda.memcpy_htod(d_input, batch)
        #     # context.execute_v2(bindings)
        #     # cuda.memcpy_dtoh(output, d_output)
        #     # transfer predictions back
        #     cuda.memcpy_dtoh_async(output, d_output, stream)
        #     # syncronize threads
        #     stream.synchronize()
        #
        #     return output
        #
        # pred = predict(preprocessed_images)
        # indices = (-pred[0]).argsort()[:1]
        # result = list(zip(indices, pred[0][indices]))
        # cuda_img = cudaFromNumpy(cv_img, isBGR=True)
        # rgb_img = cudaAllocMapped(width = cuda_img.width, height = cuda_img.height, format = 'rgb8')
        # cudaConvertColor(cuda_img,rgb_img)
        # self.output.Render(rgb_img)
        # self.output.SetStatus(f"Visual Terrain Classification | {result}")

        # using jetson-inference
        # # cv to CUDA
        # cuda_img = cudaFromNumpy(cv_img,isBGR=True)
        # rgb_img = cudaAllocMapped(width = cuda_img.width, height = cuda_img.height, format = 'rgb8')
        # cudaConvertColor(cuda_img, rgb_img)
        # self.get_logger().info(f'CUDA Image Shape: {cuda_img.width}, {cuda_img.height}')
        # #cudaDeviceSynchronize()
        # # CUDA img input
        # detections = self.mynet.Detect(rgb_img, overlay=self.detectOverlays)
        # # class_id, confidence = self.mynet.Classify(rgb_img)
        # #self.get_logger().info(f'From processor steer{steer} speed{speed}')
        # self.get_logger().info(f'detected {len(detections)} objects in image')
        # self.output.Render(rgb_img)
        # self.output.SetStatus("Object Detection | Network {:.0f} FPS".format(self.mynet.GetNetworkFPS()))

        # self.get_logger().info(f'classified {self.mynet.GetClassLabel(class_id)}')
        msg = Control()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # 设置当前时间为时间戳
        msg.header.frame_id = "None"  # 记录传感器是在哪个坐标系下采集的
        msg.speed = speed  # 不需要显示的类型转换即可保证正确性
        msg.steer = steer
        # self.publisher_.publish(msg)

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

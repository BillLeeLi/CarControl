import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Header
from hw1_msgs.msg import Deadreckoning  # 导入自定义消息
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import csv
import os
import math

M_PI = 3.14159265358979323846


class Answer(Node):
    def __init__(self):
        super().__init__("answer")

        #########################################################################
        ############## 请在此处补充订阅航位推算话题和发布里程计话题的代码 ###########
        ############## 航位推算话题名称：/hw1/deadreckoning_data #################
        ############## 航位推算话题类型：Deadreckoning ###########################
        ############## 发布的里程计话题名称：/hw1/odom ############################
        #########################################################################

        self.pub = self.create_publisher(
            Odometry, "/hw1/odom", 10
        )  # 创建发布者，话题类型为Odometry，话题名称为/hw1/odom，消息队列长度为10

        self.sub = self.create_subscription(
            Deadreckoning, "/hw1/deadreckoning_data", self.call_back, 10
        )  # 创建Deadreckoning类型话题接收者，接收话题名称/hw1/deadreckoning_data，回调函数self.call_back。10为通信参数，可以当作定值用

        #########################################################################
        #########################################################################

        self.lastori = 0  # 上一时刻朝向
        self.distance = 0  # 前进总距离

        # 上一时刻位置
        self.lastx = 0
        self.lasty = 0

    def call_back(self, msg):

        #########################################################################
        ######################## 通过航位推算法更新当前位置 #######################
        #########################################################################

        curori = msg.cur_ori  # 提取当前朝向
        distance = msg.distance  # 提取这一小段机器人的移动距离
        theta_hat = (self.lastori + curori) / 2

        self.lastx += distance * math.cos(theta_hat)  # 两帧平均的角度计算新的x,y
        self.lasty += distance * math.sin(theta_hat)

        #########################################################################
        #########################################################################

        # 更新成员变量
        self.distance += msg.distance
        self.lastori = curori

        # 打包为ros2标准里程计格式
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(curori / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(curori / 2.0)
        odom_msg.pose.pose.position.x = self.lastx
        odom_msg.pose.pose.position.y = self.lasty
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        #########################################################################
        ########################## 发布打包好的里程计话题 #########################
        #########################################################################

        self.pub.publish(odom_msg)  # 发布Deadreckoning类型话题实例，dead_msg
        #########################################################################
        #########################################################################


def main(args=None):

    rclpy.init()
    answer = Answer()
    rclpy.spin(answer)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

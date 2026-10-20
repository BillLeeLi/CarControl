import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import tf_transformations
from message_filters import Subscriber, ApproximateTimeSynchronizer

class GridMapping(Node):
    def __init__(self):
        super().__init__('grid_voting')
        
        # 订阅话题
        self.odom_sub = Subscriber(self, Odometry, '/encoder_imu_node/odom')
        self.scan_sub = Subscriber(self, LaserScan, '/urg_node/scan')
        self.imu_sub = Subscriber(self, Imu, '/encoder_imu_node/cartoimu')
        

        # 使用ApproximateTimeSynchronizer进行同步
        self.ts = ApproximateTimeSynchronizer([self.odom_sub, self.scan_sub, self.imu_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.sync_callback)

        # 地图发布
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        # 轨迹发布
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.path = Path()
        self.path.header.frame_id = 'map'

        # 栅格地图参数
        self.map_size_x = 10  # 1m x 1m (0.1m 分辨率)
        self.map_size_y = 10
        self.resolution = float(0.1)  # 每格 0.1 米
        self.map = np.zeros((self.map_size_y, self.map_size_x), dtype=np.int32)
        self.origin = (0, 0)
        self.orientation = 0.0
        
        # # 机器人在 map 坐标系中的偏置
        # self.bias = 1

    def expand_map(self, x, y):
        if x < 0 or y < 0 or x >= self.map_size_x or y >= self.map_size_y:
            # 扩图时保留旧栅格，并给负方向的新点留出空间
            offset_x = max(0, -x + 5)
            offset_y = max(0, -y + 5)
            new_size_x = max(self.map_size_x + offset_x, x + offset_x + 10)
            new_size_y = max(self.map_size_y + offset_y, y + offset_y + 10)
            new_map = np.zeros((new_size_y, new_size_x), dtype=self.map.dtype)

            # 将旧地图复制到新地图
            new_map[offset_y:offset_y + self.map_size_y, offset_x:offset_x + self.map_size_x] = self.map

            # 更新地图
            self.map = new_map
            self.origin = (self.origin[0] + offset_x, self.origin[1] + offset_y)
            self.map_size_x = new_size_x
            self.map_size_y = new_size_y
            self.get_logger().info(f"Map expanded to {new_size_x}x{new_size_y}")
            return x + offset_x, y + offset_y
        return x, y
    
    def sync_callback(self, odom_msg, scan_msg, imu_msg):
        self.odom_callback(odom_msg)
        #self.imu_callback(imu_msg)
        self.scan_callback(scan_msg)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        _, _, self.orientation = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'map'
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path.header.stamp = msg.header.stamp
        self.path_pub.publish(self.path)
        

    def imu_callback(self, msg):
        # 提取航向角 (Z轴旋转角)
        _, _, yaw = tf_transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.orientation = yaw

    def scan_callback(self, msg):
        angle = msg.angle_min
        for i, distance in enumerate(msg.ranges):
            if msg.range_min < distance < msg.range_max:

                ######################################################################### 
                ########################## 计算简易投票的坐标转换 #########################
                #########################################################################        
                        
                # 转换激光点到机器人坐标系
                # 已知激光点离机器人距离：distance
                # 已知激光点由x轴逆时针旋转角度：angle
                laser_x = distance * math.cos(angle)
                laser_y = distance * math.sin(angle)
                
                # 旋转到全局坐标系
                # 已知全局坐标系下机器人位置为：self.x, self.y
                # 已知全局坐标系下机器人朝向为：self.orientation（即PPT中delta_p）
                global_x = self.x + laser_x * math.cos(self.orientation) - laser_y * math.sin(self.orientation)
                global_y = self.y + laser_x * math.sin(self.orientation) + laser_y * math.cos(self.orientation)
                
                ######################################################################### 
                ######################################################################### 

                # 转换成栅格坐标
                pixel_x = math.floor(self.origin[0] + global_x / self.resolution)
                pixel_y = math.floor(self.origin[1] + global_y / self.resolution)

                # 动态扩大地图
                pixel_x, pixel_y = self.expand_map(pixel_x, pixel_y)
                
                # 进行简易投票
                if 0 <= pixel_x < self.map_size_x and 0 <= pixel_y < self.map_size_y:
                    self.map[pixel_y, pixel_x] += 1

            angle += msg.angle_increment

        self.publish_map()

    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.resolution = self.resolution
        grid.info.width = self.map_size_x
        grid.info.height = self.map_size_y
        grid.info.origin.position.x = -self.origin[0] * self.resolution
        grid.info.origin.position.y = -self.origin[1] * self.resolution
        grid.info.origin.orientation.w = 1.0

        # 将计数转换为占据概率
        map_flat = self.map.flatten()
        map_flat = np.clip(map_flat, 0, 100)
        grid.data = map_flat.tolist()

        self.map_pub.publish(grid)

        self.get_logger().info("Publishing Map")
        
def main(args=None):
    rclpy.init(args=args)
    node = GridMapping()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Header
from hw1_msgs.msg import Deadreckoning  # 导入自定义消息
from ament_index_python.packages import get_package_share_directory
import csv
import os

class CsvPublisher(Node):
    def __init__(self):
        super().__init__('csv_publisher')
        self.publisher_ = self.create_publisher(Deadreckoning, '/hw1/deadreckoning_data', 10)
        timer_period = 1/30  # 发布间隔时间（秒）
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.csv_path = os.path.join(get_package_share_directory('hw1'),
            'data',
            'data.csv')
        self.csv_data = self.read_csv(self.csv_path)
        self.index = 0

    def read_csv(self, file_path):
        data = []
        with open(file_path, mode='r') as file:
            csv_reader = csv.DictReader(file)
            for row in csv_reader:
                data.append(row)
        return data

    def timer_callback(self):
        if self.index < len(self.csv_data):
            row = self.csv_data[self.index]
            msg = Deadreckoning()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()  # 设置当前时间为时间戳
            msg.header.frame_id = "stm32"# 记录传感器是在哪个坐标系下采集的
            msg.cur_ori = float(row['cur_ori'])
            msg.distance = float(row['distance'])
            self.publisher_.publish(msg)
            #self.get_logger().info(f'Publishing: cur_ori={msg.cur_ori}, distance={msg.distance}')
            self.index += 1
        else:
            self.get_logger().info('All data published.')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    csv_publisher = CsvPublisher()
    rclpy.spin(csv_publisher)
    csv_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
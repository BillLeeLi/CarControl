import rclpy
from rclpy.node import Node
import csv
from sensor_msgs.msg import Imu
from robotsdk_msgs.msg import ImuEncoderAngle, AllSensor
from rclpy.serialization import deserialize_message
import rosbag2_py

class Bag2CsvImu(Node):
    def __init__(self, bag_file, topic_name, csv_file):
        super().__init__('bag2csv_imu')
        self.bag_file = bag_file
        self.topic_name = topic_name
        self.csv_file = csv_file
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_ori = 0.0
        self.distance = 0.0

    def process_bag(self):
        storage_options = rosbag2_py.StorageOptions(uri=self.bag_file, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['last_ori', 'distance'])

            while reader.has_next():
                (topic, data, t) = reader.read_next()
                if topic == self.topic_name:
                    imu_msg = deserialize_message(data, ImuEncoderAngle)
                    self.calculate_and_write(imu_msg, writer)

    def calculate_and_write(self, imu_msg, writer):
        # 假设last_x, last_y, last_ori是从IMU消息中提取的
        current_x = imu_msg.x
        current_y = imu_msg.y
        current_ori = imu_msg.orientation  # 这里只是一个示例，实际计算可能不同

        # 计算distance和last_ori
        self.distance = ((current_x - self.last_x) ** 2 + (current_y - self.last_y) ** 2) ** 0.5
        self.last_ori = current_ori

        # 写入CSV
        writer.writerow([self.last_ori, self.distance])

        # 更新last_x, last_y, last_ori
        self.last_x = current_x
        self.last_y = current_y
        self.last_ori = current_ori

def main(args=None):
    rclpy.init(args=args)
    bag_file = '/data/for_carto_like2'
    topic_name = '/encoder_imu_node/imuencoder'
    csv_file = 'output.csv'
    bag2csv_imu = Bag2CsvImu(bag_file, topic_name, csv_file)
    bag2csv_imu.process_bag()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
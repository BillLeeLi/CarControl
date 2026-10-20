# #!/home/poss/miniconda3/envs/test0/bin/python
# import os
# os.environ['PYTHONPATH'] = '/home/poss/miniconda3/envs/test0/lib/python3.8/site-packages:' + os.environ.get('PYTHONPATH', '')
import rclpy
import struct
import serial
from rclpy.node import Node
import serial.tools.list_ports
import time
import math
import numpy as np
from robotsdk_msgs.msg import Control, ImuEncoderAngle
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
# import arrow


M_PI = 3.14159265358979323846

class Encoder_IMU_Node(Node):
    def __init__(self):
        super().__init__('encoder_imu_node')

        #########################################################################################
        ################### zht:param and initialaziton of encoder_imu for publish##############
        #########################################################################################
        self.port = '/dev/ttyUSB0'
        self.baudrate = 38400
        self.bytesize = 8   # serial.EIGHTBITS
        self.parity = 'N'   # serial.PARITY_NONE
        self.stopbits = 1   # serial.STOPBITS_ONE
        self.flowtype = 0
        self.recvpacksize = 1
        self.packhead = 0xa2
        self.packtail = 0x2a
        #self.distancePerPulse = 0.00001117
        self.distancePerPulse = 0.00000611
        self.maxPulse = 30000
        self.initx = 0
        self.inity = 0
        self.initori = 0
        self.time_period = 0.02
        self.controlTopic = '/process_node/control'


        self.declare_parameter('port', self.port)
        self.declare_parameter('baudrate', self.baudrate)
        self.declare_parameter('bytesize', self.bytesize)
        self.declare_parameter('parity', self.parity)
        self.declare_parameter('stopbits', self.stopbits)
        self.declare_parameter('flowtype', self.flowtype)
        self.declare_parameter('recvpacksize', self.recvpacksize)
        self.declare_parameter('packhead', self.packhead)
        self.declare_parameter('packtail', self.packtail)
        self.declare_parameter('distancePerPulse', self.distancePerPulse)
        self.declare_parameter('maxPulse', self.maxPulse)
        self.declare_parameter('initx', self.initx)
        self.declare_parameter('inity', self.inity)
        self.declare_parameter('initori', self.initori)
        self.declare_parameter('time_period', self.time_period)
        self.declare_parameter('controlTopic', self.controlTopic)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.bytesize = self.get_parameter('bytesize').get_parameter_value().integer_value
        self.parity = self.get_parameter('parity').get_parameter_value().string_value
        self.stopbits = self.get_parameter('stopbits').get_parameter_value().integer_value
        self.flowtype = self.get_parameter('flowtype').get_parameter_value().integer_value
        self.recvpacksize = self.get_parameter('recvpacksize').get_parameter_value().integer_value
        self.packhead = self.get_parameter('packhead').get_parameter_value().integer_value
        self.packtail = self.get_parameter('packtail').get_parameter_value().integer_value
        self.distancePerPulse = self.get_parameter('distancePerPulse').get_parameter_value().double_value
        self.maxPulse = self.get_parameter('maxPulse').get_parameter_value().integer_value
        self.initx = self.get_parameter('initx').get_parameter_value().double_value
        self.inity = self.get_parameter('inity').get_parameter_value().double_value
        self.initori = self.get_parameter('initori').get_parameter_value().double_value
        self.time_period = self.get_parameter('time_period').get_parameter_value().double_value
        self.controlTopic = self.get_parameter('controlTopic').get_parameter_value().string_value
        self.timer = self.create_timer(self.time_period, self.timer_callback)

        self.odometry = 0.0
        self.lastx = 0.0
        self.lasty = 0.0
        self.lastori = 0.0
        self.lastpulsenum = -1
        self.lastspeed = 0.0
        self.backDis = 0
        self.isInit = False
        self.initOriValue = 0
        self.lastTimestamp = None

        #######################################################################
        ################### yzy:safe check,open serial port####################
        #######################################################################
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            self.get_logger().info(f"Port available: {port.device}, Description: {port.description}")
        if not ports:
            self.get_logger().info("No serial ports found!")
        self.get_logger().info(f"Our Port is {self.port}")
        self.get_logger().info(f"Our Parity is {self.parity}")
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=self.bytesize,
                parity=self.parity,
                stopbits=self.stopbits,
                timeout=5,
            )
            self.get_logger().info("Serial port initialized successfully.")
        except serial.SerialException as e:
            self.get_logger().info(f"Failed to open serial port: {e}")
        # yzy:until now ser is successful ini ,but still need to be tested as below
        self.get_logger().info("Testing serial...")
        try:
            start_time = time.time()
            while True:
                data = self.ser.read(1)  # 尝试读取一个字节
                if data:
                    self.get_logger().info("Testing passed！")
                    break
                elif time.time() - start_time >= 0.5:
                    # 超过3秒没有数据，重启串口
                    self.get_logger().info("No data received for 0.5 seconds, restarting serial port...")
                    self.ser.close()  # 关闭串口
                    time.sleep(1)  # 稍等1秒
                    self.ser.open()  # 重新打开串口
                    self.get_logger().info("Serial port restarted.")
                    start_time = time.time()  # 重置计时器
        except KeyboardInterrupt:
            self.get_logger().info("Program terminated by user.")

        #######################################################################
        ################### yzy:buffer initialization##########################
        #######################################################################
        self.buffer = bytearray()


        #######################################################################
        ################### yzy:publish initialization#########################
        #######################################################################
        self.publisher_ = self.create_publisher(ImuEncoderAngle, 'imuencoder', 10)
        self.carto_pub = self.create_publisher(Imu, 'cartoimu', 10)
        self.nav2_odom = self.create_publisher(Odometry, 'odom', 10)

        self.get_logger().info("Create encoder_imu publisher successfully!")


        #######################################################################
        ################### yzy:subscript initialization#######################
        #######################################################################
        self.subscription = self.create_subscription(
            Control,
            self.controlTopic,
            self.listener_callback,
            1000)
        self.subscription  # 防止未使用变量警告

        # self.control_timer = self.create_timer(1, self.control_listener_callback)
        # self.control_steer = np.arange(-400, 400, 50)
        # self.control_idx = 0

    def process_packet(self, packet):
        shorts = struct.unpack('<5h', packet[1:11])
        # self.get_logger().info(f"Latest valid packet shorts: {shorts}")
        msg = ImuEncoderAngle()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # 设置当前时间为时间戳
        msg.header.frame_id = "stm32"# 记录传感器是在哪个坐标系下采集的
        #process_data to msg zzy
        msg.sanglex, msg.sangley, msg.sanglez, msg.spulsenum, msg.sbackdis = shorts  # 直接依顺序从接受信号shorts赋值到5个变量
        if self.lastTimestamp is None:
            self.lastTimestamp = msg.header.stamp
        # duration between current timestamp and last one (in seconds)
        deltaTime = ((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) -
                     (self.lastTimestamp.sec + self.lastTimestamp.nanosec * 1e-9))  # 计算两帧时间差，如果太小，speed会不计算。
        self.lastTimestamp = msg.header.stamp # Q1 added by zht
        curori = M_PI * msg.sanglez / 1800.0  # 和gb一致
        isEStop = (msg.sanglez == 0) and (msg.spulsenum == 0)  # 和gb一致
        if isEStop:  # 和gb一致
            self.lastpulsenum = 0
            self.initOriValue = - self.lastori
        if not self.isInit:  # 和gb一致
            self.initOriValue = curori
            self.isInit = True
        curori = curori - self.initOriValue  # 和gb一致
        aveori = (self.lastori + curori) / 2.0 + M_PI if math.fabs(self.lastori - curori) > M_PI \
            else (self.lastori + curori) / 2.0  # 两帧差距过大，补加pi等价于较小一个值补加2*pi

        mylastori = self.lastori
        self.lastori = curori  # 和gb一致
        if self.lastpulsenum == -1:  # 和gb一致
            self.lastpulsenum = msg.spulsenum
        deltaP = msg.spulsenum - self.lastpulsenum
        if deltaP < - self.maxPulse / 2:  # 和gb一致，一旦正走突变30000->0 deltaP加回30000
            deltaP += self.maxPulse
        elif deltaP > self.maxPulse / 2:  # 和gb一致，一旦反走突变0->30000 deltaP减回30000
            deltaP -= self.maxPulse
        else:
            pass
        distance = deltaP * self.distancePerPulse
        self.lastpulsenum = msg.spulsenum
        self.odometry = distance + self.odometry  # 是计算了微小位移的累积值，甚至倒车会减  走直线的时候odemetry比较准

        self.lastx += math.cos(aveori) * distance  # 两帧平均的角度计算新的x,y
        self.lasty += math.sin(aveori) * distance

        msg.speed = self.lastspeed if deltaTime < 1e-5 else distance / deltaTime  # 如果两帧间隔过短，速度沿用上一帧速度，否则计算位移除以时间
        msg.odometry = self.odometry
        msg.orientation = self.lastori
        msg.x = self.lastx
        msg.y = self.lasty

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = math.sin(msg.orientation / 2.0)
        imu_msg.orientation.w = math.cos(msg.orientation / 2.0)

        imu_msg.linear_acceleration.x = (msg.speed - self.lastspeed) / deltaTime * math.cos(aveori) if deltaTime > 1e-5 else 0.0
        imu_msg.linear_acceleration.y = (msg.speed - self.lastspeed) / deltaTime * math.sin(aveori) if deltaTime > 1e-5 else 0.0
        imu_msg.linear_acceleration.z = 9.81

        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        vz = (-mylastori + curori)  + M_PI if math.fabs(mylastori - curori) > M_PI \
            else (-mylastori + curori)   # 两帧差距过大，补加pi等价于较小一个值补加2*pi
        imu_msg.angular_velocity.z = vz / deltaTime if deltaTime > 1e-5 else 0.0

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.orientation = imu_msg.orientation
        odom_msg.pose.pose.position.x = msg.x
        odom_msg.pose.pose.position.y = msg.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.twist.twist.linear.x = (msg.speed - self.lastspeed) * math.cos(aveori) if deltaTime > 1e-5 else 0.0
        odom_msg.twist.twist.linear.y = (msg.speed - self.lastspeed) * math.sin(aveori) if deltaTime > 1e-5 else 0.0
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = imu_msg.angular_velocity.x
        odom_msg.twist.twist.angular.y = imu_msg.angular_velocity.y
        odom_msg.twist.twist.angular.z = imu_msg.angular_velocity.z

        self.carto_pub.publish(imu_msg)
        self.nav2_odom.publish(odom_msg)

        self.lastspeed = msg.speed


        self.publisher_.publish(msg)







    def timer_callback(self):
        try:
            # yzy:Judge the if data enough to be read
            if self.ser.in_waiting < 12:
                return
            data = self.ser.read(self.ser.in_waiting)
            if data:
                self.buffer.extend(data) # all messages in buffer
                # yzy: read from the tail to acc
                startId=len(self.buffer)-11
                while(startId:=startId-1)>=0:
                    if (self.buffer[startId+11]==0x2A and self.buffer[startId]==0xA2):
                        packet = self.buffer[startId:startId+12] # yzy:此时头尾标识字节也一并读入
                        self.process_packet(packet)
                        self.buffer = self.buffer[startId+12:]
                        return
                self.buffer.clear()


        except KeyboardInterrupt:
            if self.ser.is_open:
                self.ser.close()
            self.get_logger().info("Program terminated by user. Ser is closed.")

    def listener_callback(self, msg):
        # yzy:处理接收到的control信息
        # self.get_logger().info('I heard:{msg.steer}{msg.speed}')
        packed_data = struct.pack('<BBhhB', 0xF8, 4, msg.steer, msg.speed, 0x8F)
        if self.ser.is_open:
            self.ser.write(packed_data)

    # def control_listener_callback(self):
    #     fake_control_steer = self.control_steer[self.control_idx]
    #     self.control_idx = (self.control_idx + 1) % len(self.control_steer)
    #     self.get_logger().info(f'send steer: {fake_control_steer}')
    #     packed_data = struct.pack('<BBhhB', 0xF8, 4, fake_control_steer, 100, 0x8F)
    #     if self.ser.is_open:
    #         self.ser.write(packed_data)


    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        self.get_logger().info("Close imu node successfully!")
        return super().destroy_node()


def main(args=None):

    rclpy.init()
    encoder_imu_node = Encoder_IMU_Node()
    # try:
    #     rclpy.spin(encoder_imu_node)
    # except:
    #     encoder_imu_node.destroy_node()
    # finally:
    #     rclpy.shutdown()
    rclpy.spin(encoder_imu_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

std_msgs/Header header  # Header 包含时间戳和帧ID
# original stm32 sensor data
int16 sanglex
int16 sangley
int16 sanglez
int16 spulsenum
int16 sbackdis

# xtion data
sensor_msgs/Image depth
sensor_msgs/Image xtion_color

# csi data
# sensor_msgs/Image csi_color

#ladar data
sensor_msgs/LaserScan urg

# processed data copy from gaobiao
float64 x
float64 y
float64 orientation
float64 speed
float64 odometry

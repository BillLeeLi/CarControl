//
// Created by poss on 24-9-2.
//
//
// Created by poss on 24-4-17.
//

//#define DEBUG
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include "cv_bridge/cv_bridge.h"
#include <sys/stat.h>
#include "std_msgs/msg/header.hpp"
#include "robotsdk_msgs/msg/all_sensor.hpp"
#include "robotsdk_msgs/msg/control.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "robotsdk_msgs/msg/imu_encoder_angle.hpp"
#include <functional>
#include <cmath>
//#include <pcl/io/io.h>
//#include <laser_geometry>
//#include <sensor_msgs/PointCloud2.h>

/*
*/
using std::string;
class Process_node_cpp : public rclcpp::Node
{

public:
    // 构造函数,有一个参数为节点名称
    explicit Process_node_cpp() : Node("process_node_cpp")
    {
            // default values for topic name
    string xtionColorTopic = "/xtion_node/color";
    string xtionDepthTopic = "/xtion_node/depth";
    //string csiColorTopic = "/csi_node/raw";
    string urgScanTopic = "/scan";
    string imuEncoderTopic = "/encoder_imu_node/imuencoder";
    string imuTopic = "/encoder_imu_node/cartoimu";
    string odomTopic = "/encoder_imu_node/odom";
    string allsensorTopic = "/sync_store_node/allsensor";
    string savePath = "/home/poss/data";
        // declare parameters
        this->declare_parameter("xtionColorTopic", xtionColorTopic);
        this->declare_parameter("xtionDepthTopic", xtionDepthTopic);
        //this->declare_parameter("csiColorTopic", csiColorTopic);
        this->declare_parameter("urgScanTopic", urgScanTopic);
        this->declare_parameter("imuEncoderTopic", imuEncoderTopic);
        this->declare_parameter("savePath", savePath);
        this->declare_parameter("imuTopic", imuTopic);
        this->declare_parameter("odomTopic", odomTopic);
        this->declare_parameter("allsensorTopic", allsensorTopic);
        // retrieve parameters
        this->get_parameter("xtionColorTopic", xtionColorTopic);
        this->get_parameter("xtionDepthTopic", xtionDepthTopic);
        //this->get_parameter("csiColorTopic", csiColorTopic);
        this->get_parameter("urgScanTopic", urgScanTopic);
        this->get_parameter("imuEncoderTopic", imuEncoderTopic);
        this->get_parameter("savePath", savePath);
        this->get_parameter("imuTopic", imuTopic);
        this->get_parameter("odomTopic",odomTopic);
        this->get_parameter("allsensorTopic", allsensorTopic);


        // in create_subscription, the template deduces the first argument of the call_back function. for lambda and std::bind, they perfect forward the argument of message type while others not.
        sub_sensor = this->create_subscription<robotsdk_msgs::msg::AllSensor>(allsensorTopic, 10, [this](robotsdk_msgs::msg::AllSensor::SharedPtr PH1) { laser_scan_callback(std::forward<decltype(PH1)>(PH1)); });
        pub_ctrl = this->create_publisher<robotsdk_msgs::msg::Control>("/process_node/control",10);
    }

private:
// 声明一个订阅者（成员变量）
    rclcpp::Subscription<robotsdk_msgs::msg::AllSensor>::SharedPtr sub_sensor;

    rclcpp::Publisher<robotsdk_msgs::msg::Control>::SharedPtr pub_ctrl;

    // 成员变量 for midterm
    double error = 0.0;
    double preverror = 0.0;
    double integral = 0.0;
    double kp = 100;
    double ki = 0.1;
    double kd = 0.1;
    int    avoid_block = 0;
    int    prev_avoid_block = 0;
    int    avoid_direction = 0;
    short  pre_steer = 0;
    int    back_flag = 0;
    int    back_time = 40;


    // 收到话题数据的回调函数
    void laser_scan_callback(const robotsdk_msgs::msg::AllSensor::SharedPtr msg)
    {
        auto ranges = msg->urg.ranges;// 0~180

        // Your codes here
        int speed = 100;
        int steer = 100;
        
        auto pub_msg = robotsdk_msgs::msg::Control();
        pub_msg.header = std_msgs::msg::Header();
        pub_msg.header.stamp = this->get_clock()->now();  // 设置当前时间为时间戳
        pub_msg.header.frame_id = "None";  // 记录传感器是在哪个坐标系下采集的
        pub_msg.speed = speed;  // 不需要显示的类型转换即可保证正确性
        pub_msg.steer = steer;
        cv::Mat image = cv_bridge::toCvCopy(msg->xtion_color,msg->xtion_color.encoding)->image;

        cv::putText(image,"steer: "+std::to_string(steer)+"\nspeed: "+std::to_string(speed),
            cv::Point(40,15),0,2,{255,255,255});

        cv::imshow("color",image);
        cv::waitKey(1);

        pub_ctrl->publish(pub_msg);
    };


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Process_node_cpp>();
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

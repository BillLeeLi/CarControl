 //
// Created by poss on 24-4-17.
//

//#define DEBUG
#include "rclcpp/rclcpp.hpp"
#include "sync_store_node/sync_store_node.h"
#include "../include/sync_store_node/sync_store_node.h"
#include <opencv2/opencv.hpp>
#include <string>
#include "cv_bridge/cv_bridge.h"
#include <sys/stat.h>
#include "std_msgs/msg/header.hpp"
//#include <pcl/io/io.h>
//#include <laser_geometry>
//#include <sensor_msgs/PointCloud2.h>

Sync_Store_Node::Sync_Store_Node(): Node("sync_store")
{

    // default values for topic names
    xtionColorTopic = "/xtion_node/color";
    xtionDepthTopic = "/xtion_node/depth";
    //csiColorTopic = "/csi_node/raw";
    urgScanTopic = "/urg_node/scan";
    imuEncoderTopic = "/encoder_imu_node/imuencoder";
    savePath = "/home/poss/data";
    // declare parameters
    this->declare_parameter("xtionColorTopic", xtionColorTopic);
    this->declare_parameter("xtionDepthTopic", xtionDepthTopic);
    //this->declare_parameter("csiColorTopic", csiColorTopic);
    this->declare_parameter("urgScanTopic", urgScanTopic);
    this->declare_parameter("imuEncoderTopic", imuEncoderTopic);
    this->declare_parameter("savePath", savePath);
    // retrieve parameters
    this->get_parameter("xtionColorTopic", xtionColorTopic);
    this->get_parameter("xtionDepthTopic", xtionDepthTopic);
    //this->get_parameter("csiColorTopic", csiColorTopic);
    this->get_parameter("urgScanTopic", urgScanTopic);
    this->get_parameter("imuEncoderTopic", imuEncoderTopic);
    this->get_parameter("savePath", savePath);
    RCLCPP_INFO(this->get_logger(), "savepath is: %s", savePath.c_str());
    // initialize subscribers
    xtionColorSubscriber_.subscribe(this, xtionColorTopic);
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic '%s'", xtionColorTopic.c_str());
    xtionDepthSubscriber_.subscribe(this, xtionDepthTopic);
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic '%s'", xtionDepthTopic.c_str());
    //csiColorSubscriber_.subscribe(this, csiColorTopic);
    //RCLCPP_INFO(this->get_logger(), "Subscribed to topic '%s", csiColorTopic.c_str());
    urgScanSubscriber_.subscribe(this, urgScanTopic);
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic '%s'", urgScanTopic.c_str());
    imuEncoderSubscriber_.subscribe(this, imuEncoderTopic);
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic '%s", imuEncoderTopic.c_str());
    // initialize publishers
    xtionColorPublisher_ = this->create_publisher<sensor_msgs::msg::Image>("xtion_color_sync", 10);
    xtionDepthPublisher_ = this->create_publisher<sensor_msgs::msg::Image>("xtion_depth_sync", 10);
    //csiColorPublisher_ = this->create_publisher<sensor_msgs::msg::Image>("csi_raw_sync", 10);
    urgScanPublisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("urg_scan_sync", 10);
    imuEncoderPublisher_ = this->create_publisher<robotsdk_msgs::msg::ImuEncoderAngle>("imu_encoder_sync", 10);
    allSensorPublisher_ = this->create_publisher<robotsdk_msgs::msg::AllSensor>("allsensor", 10);
    // create directory named with timestamp
    rclcpp::Clock::SharedPtr clock = this->get_clock();
    int second_init = clock->now().seconds();
    folderPath = savePath + "/" + std::to_string(second_init);
    std::string command;
    struct stat info;
    if (stat(folderPath.c_str(), &info) != 0) {
        command = "mkdir " + folderPath;
        system(command.c_str());
        command = "mkdir " + folderPath + "/xtioncolor";
        system(command.c_str());
        command = "mkdir " + folderPath + "/depth";
        //system(command.c_str());
        //command = "mkdir " + folderPath + "/csicolor";
        system(command.c_str());
        command = "mkdir " + folderPath + "/urg";
        system(command.c_str());
        command = "mkdir " + folderPath + "/imu";
        system(command.c_str());
        RCLCPP_INFO(this->get_logger(), "Data saved to %s", folderPath.c_str());
    }
    else{
        RCLCPP_INFO(this->get_logger(), "failed to save data, the path %s has already exists!", folderPath.c_str());
    }

    // message filters
    Sync.reset(new message_filters::Synchronizer<syncPolicy>(syncPolicy(10),
            xtionColorSubscriber_, xtionDepthSubscriber_, urgScanSubscriber_, imuEncoderSubscriber_));
    Sync->registerCallback(&Sync_Store_Node::callback, this);
}

void Sync_Store_Node::callback(const sensor_msgs::msg::Image::SharedPtr xtionColorPtr,
                               const sensor_msgs::msg::Image::SharedPtr xtionDepthPtr,
                               const sensor_msgs::msg::LaserScan::SharedPtr urgScanPtr,
                               const robotsdk_msgs::msg::ImuEncoderAngle::SharedPtr imuEncoderPtr)
{
//    RCLCPP_INFO(this->get_logger(), "Recevied synchronized message!");
    xtionColorPublisher_->publish(*xtionColorPtr);
    xtionDepthPublisher_->publish(*xtionDepthPtr);
    //csiColorPublisher_->publish(*csiColorPtr);
    urgScanPublisher_->publish(*urgScanPtr);
    imuEncoderPublisher_->publish(*imuEncoderPtr);

    //整合所有数据发布allsensor话题
    auto allSensorMsg = robotsdk_msgs::msg::AllSensor();
    allSensorMsg.xtion_color = *xtionColorPtr;
    allSensorMsg.depth = *xtionDepthPtr;
    //allSensorMsg.csi_color = *csiColorPtr;
    allSensorMsg.urg = *urgScanPtr;
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();  // 设置当前时间为时间戳
    header.frame_id = "sync_node";  // 设置坐标系
    allSensorMsg.header = header;
    allSensorMsg.sanglex = imuEncoderPtr->sanglex;
    allSensorMsg.sangley = imuEncoderPtr->sangley;
    allSensorMsg.sanglez = imuEncoderPtr->sanglez;
    allSensorMsg.spulsenum = imuEncoderPtr->spulsenum;
    allSensorMsg.sbackdis = imuEncoderPtr->sbackdis;
    allSensorMsg.x = imuEncoderPtr->x;
    allSensorMsg.y = imuEncoderPtr->y;
    allSensorMsg.orientation = imuEncoderPtr->orientation;
    allSensorMsg.speed = imuEncoderPtr->speed;
    allSensorMsg.odometry = imuEncoderPtr->odometry;
    allSensorPublisher_->publish(allSensorMsg);

//TODO: add code to store images to disk here.
    int second = xtionColorPtr->header.stamp.sec;
    int nanosecond = xtionColorPtr->header.stamp.nanosec;
//xtioncolor, depth, csicolor are saved in different directories named with timestamp
    std::string xtionColorSavePath = folderPath + "/xtioncolor/" + std::to_string(second) + std::to_string(nanosecond) + ".jpg";
    std::string depthSavePath = folderPath + "/depth/" + std::to_string(second) +std::to_string(nanosecond) + ".tif";
    //std::string csiColorSavePath = folderPath + "/csicolor/" + std::to_string((second)) + std::to_string((nanosecond)) + ".jpg";
    cv_bridge::CvImagePtr xtionColor_ptr = cv_bridge::toCvCopy(xtionColorPtr, sensor_msgs::image_encodings::BGR8);
    #ifdef DEBUG
    cv::imwrite(xtionColorSavePath, xtionColor_ptr->image);
    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(xtionDepthPtr, sensor_msgs::image_encodings::TYPE_32FC1);
    cv::imwrite(depthSavePath, depth_ptr->image);
    //cv_bridge::CvImagePtr csiColor_ptr = cv_bridge::toCvCopy(csiColorPtr, sensor_msgs::image_encodings::BGR8);
    //cv::imwrite(csiColorSavePath, csiColor_ptr->image);
    #endif
//save pointcloud to pcd file
//    std::string urgSavePath = folderPath + "/urg/" + std::to_string(second) + std::to_string(nanosecond) + ".pcd";
//    sensor_msgs::msgs::PointCloud2 cloud_msg;
//    laser_geometry::LaserProjection projector;
//    projector.projectLaser(*urgScanPtr, cloud_msg);
//    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
//    pcl::fromROSMsg(cloud_msg, pcl_cloud);
//    pcl::io::savePCDFileBinary(urgSavePath, pcl_cloud);

//
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sync_Store_Node>());
    rclcpp::shutdown();
    return 0;
}

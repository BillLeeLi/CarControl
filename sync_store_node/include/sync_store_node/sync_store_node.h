//
// Created by poss on 24-4-17.
//

#ifndef SYNC_STORE_NODE_H
#define SYNC_STORE_NODE_H
#include "rclcpp/rclcpp.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "robotsdk_msgs/msg/all_sensor.hpp"
#include "robotsdk_msgs/msg/imu_encoder_angle.hpp"
//add more types below

class Sync_Store_Node : public rclcpp::Node
{
public:
    Sync_Store_Node();
    void callback(const sensor_msgs::msg::Image::SharedPtr xtionColorPtr,
                  const sensor_msgs::msg::Image::SharedPtr xtionDepthPtr,
                  const sensor_msgs::msg::LaserScan::SharedPtr urgScanPtr,
                  const robotsdk_msgs::msg::ImuEncoderAngle::SharedPtr imuEncoderPtr);
private:
    // image publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr xtionColorPublisher_, xtionDepthPublisher_;
    // urg laser scan publisher
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr urgScanPublisher_;
    rclcpp::Publisher<robotsdk_msgs::msg::ImuEncoderAngle>::SharedPtr imuEncoderPublisher_;
    rclcpp::Publisher<robotsdk_msgs::msg::AllSensor>::SharedPtr allSensorPublisher_;
    // message filters
    message_filters::Subscriber<sensor_msgs::msg::Image> xtionColorSubscriber_, xtionDepthSubscriber_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> urgScanSubscriber_;
    message_filters::Subscriber<robotsdk_msgs::msg::ImuEncoderAngle> imuEncoderSubscriber_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                            sensor_msgs::msg::Image,
                                                            sensor_msgs::msg::LaserScan,
                                                            robotsdk_msgs::msg::ImuEncoderAngle> syncPolicy;
    std::shared_ptr<message_filters::Synchronizer<syncPolicy>> Sync;
    std::string xtionColorTopic, xtionDepthTopic,urgScanTopic, imuEncoderTopic, savePath, folderPath;
};

#endif //SYNC_STORE_NODE_H

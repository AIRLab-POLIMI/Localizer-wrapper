#ifndef ARTSLAM_LOCALIZER_HPP
#define ARTSLAM_LOCALIZER_HPP

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "localization/localizer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace lots::localization;

class ArtslamLocalizer : public rclcpp::Node
{
public:
    ArtslamLocalizer();

private:
    Localizer localizer_;
    
    std::string imu_topic_;
    std::string pointcloud_topic_;
    std::string odom_topic_;
    std::string gnss_topic_;
    std::string global_frame_;
    std::string base_frame_;
    std::string sensor_frame_;

    bool use_imu_ = false;
    bool use_encoders_ = false;
    bool use_gnss_ = false;
    bool use_point_clouds_ = false;
    bool base_tf_ready_ = false;

    uint imu_msg_seq_;
    uint pointcloud_msg_seq_;
    double predicted_x_ = 0.0;
    double predicted_y_ = 0.0;
    double corrected_x_ = 0.0;
    double corrected_y_ = 0.0;

    double loc_time_ = 0.0;
    double loc_count_ = 0.0;

    Odometry_MSG::Ptr first_odom = nullptr;

    tf2::Transform sensor_to_base_tf_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose2_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_p_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_c_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;

    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    std::mutex saving_mutex_;

    void imu_callback(const sensor_msgs::msg::Imu& msg);
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2& msg);
    void odom_callback(const nav_msgs::msg::Odometry& msg);
    void gnss_callback(const sensor_msgs::msg::NavSatFix& msg);
    void odom_timer_callback();
    void state_timer_callback();
    void check_sensor_transform();
};

#endif /* ARTSLAM_LOCALIZER_HPP */

#include "localizer_wrapper/ArtslamLocalizer.hpp"
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <lots_utils/types_converter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace lots::localization;
using namespace lots::core::utils;
using namespace lots::core::types;
using namespace std::chrono_literals;

//const std::string graph_location = "/home/matteo/LOTS/bags_21-09/results_06/graph_mulran.g2o";
//const std::string clouds_location = "/home/matteo/LOTS/bags_21-09/results_06/keyframes/";
//const std::string graph_location = "/home/matteo/Datasets/Carmagnola_181023/results_01/graph_mulran.g2o";
//const std::string clouds_location = "/home/matteo/Datasets/Carmagnola_181023/results_01/keyframes/";

const std::string graph_location = "/home/matteo/risultati/graph_mulran.g2o";
const std::string clouds_location = "/home/matteo/risultati/keyframes/";
const std::string gnss_location = "/home/matteo/risultati/gps.txt";
const std::string params_location = "/home/matteo/ros2_ws/src/localizer_wrapper/config/loc_corrector_params.yaml";


ArtslamLocalizer::ArtslamLocalizer()
    : rclcpp::Node("artslam_localizer"), localizer_(graph_location, clouds_location, gnss_location, params_location)
{
    using namespace std::placeholders;

    this->declare_parameter("imu_topic", rclcpp::ParameterValue(""));
    this->get_parameter<std::string>("imu_topic", imu_topic_);
    this->declare_parameter("pointcloud_topic", rclcpp::ParameterValue(""));
    this->get_parameter<std::string>("pointcloud_topic", pointcloud_topic_);
    this->declare_parameter("odom_topic", rclcpp::ParameterValue(""));
    this->get_parameter<std::string>("odom_topic", odom_topic_);
    this->declare_parameter("global_frame", rclcpp::ParameterValue(""));
    this->get_parameter<std::string>("global_frame", global_frame_);
    this->declare_parameter("base_frame", rclcpp::ParameterValue(""));
    this->get_parameter<std::string>("base_frame", base_frame_);
    this->declare_parameter("sensor_frame", rclcpp::ParameterValue(""));
    this->get_parameter<std::string>("sensor_frame", sensor_frame_);

    this->declare_parameter("use_imu", rclcpp::ParameterValue(false));
    this->get_parameter("use_imu", use_imu_);
    this->declare_parameter("use_encoders", rclcpp::ParameterValue(false));
    this->get_parameter("use_encoders", use_encoders_);
    this->declare_parameter("use_gnss", rclcpp::ParameterValue(false));
    this->get_parameter("use_gnss", use_gnss_);
    this->declare_parameter("use_point_clouds", rclcpp::ParameterValue(false));
    this->get_parameter("use_point_clouds", use_point_clouds_);

    std::cout << std::boolalpha << "[LOCALIZER INIT] Using IMU data: " << use_imu_ << std::endl;
    std::cout << std::boolalpha << "[LOCALIZER INIT] Using Encoders data: " << use_encoders_ << std::endl;
    std::cout << std::boolalpha << "[LOCALIZER INIT] Using GNSS data: " << use_gnss_ << std::endl;
    std::cout << std::boolalpha << "[LOCALIZER INIT] Using Point clouds: " << use_point_clouds_ << std::endl;
    std::cout << std::noboolalpha;

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_,
        rclcpp::QoS(rclcpp::SensorDataQoS()),
        std::bind(&ArtslamLocalizer::imu_callback, this, _1));

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_,
        rclcpp::QoS(rclcpp::SensorDataQoS()),
        std::bind(&ArtslamLocalizer::pointcloud_callback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            rclcpp::QoS(rclcpp::SensorDataQoS()),
            std::bind(&ArtslamLocalizer::odom_callback, this, _1));

    gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix",
            rclcpp::QoS(rclcpp::SensorDataQoS()),
            std::bind(&ArtslamLocalizer::gnss_callback, this, _1));

    pose2_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/corrected_pose", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    odom_p_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/od_p", 10);
    odom_c_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/od_c", 10);

    imu_msg_seq_ = 0;
    pointcloud_msg_seq_ = 0;
    predicted_x_ = 0.0;
    predicted_y_ = 0.0;

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    odom_timer_ = this->create_wall_timer(20ms, std::bind(&ArtslamLocalizer::odom_timer_callback, this));

}

void ArtslamLocalizer::imu_callback(const sensor_msgs::msg::Imu& msg) {
    // 1. If IMU data usage is disabled, immediately return
    if(!use_imu_) return;

    // 2. Otherwise, create a new IMU custom message and convert from Imu
    IMU_MSG::Ptr imu_msg_ = std::make_shared<IMU_MSG>();

    // TODO maybe also use covariances
    imu_msg_->header_.timestamp_ = static_cast<int64_t>(msg.header.stamp.sec * 1000000000ull) + msg.header.stamp.nanosec;
    imu_msg_->header_.frame_id_ = msg.header.frame_id;
    imu_msg_->angular_velocity_.value_ << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
    imu_msg_->angular_velocity_.covariance_ = EigMatrix3d::Identity();
    imu_msg_->linear_acceleration_.value_ << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
    imu_msg_->linear_acceleration_.covariance_ = EigMatrix3d::Identity();

    //std::cout << "ANGULAR VEL -- " << imu_msg_->angular_velocity_.value_.transpose() << std::endl;
    //std::cout << "LINEAR ACCE -- " << imu_msg_->linear_acceleration_.value_.transpose() << std::endl;
    if(!use_encoders_ && !use_point_clouds_)
        imu_msg_->linear_acceleration_.value_.z() = -9.80665;
    Odometry_MSG::Ptr pred_msg = localizer_.predict(imu_msg_);

    if (pred_msg) {
        geometry_msgs::msg::TransformStamped map_sensor_tf;

        map_sensor_tf.header.stamp = msg.header.stamp;
        map_sensor_tf.header.frame_id = global_frame_;
        map_sensor_tf.child_frame_id = "os_sensor_tmp"; // TODO use sensor_frame_ (parameter) instead

        map_sensor_tf.transform.translation.x = pred_msg->value_(0, 3);
        map_sensor_tf.transform.translation.y = pred_msg->value_(1, 3);
        map_sensor_tf.transform.translation.z = pred_msg->value_(2, 3);

        tf2::Quaternion q;
        tf2::Matrix3x3(pred_msg->value_(0, 0),
                       pred_msg->value_(0, 1),
                       pred_msg->value_(0, 2),
                       pred_msg->value_(1, 0),
                       pred_msg->value_(1, 1),
                       pred_msg->value_(1, 2),
                       pred_msg->value_(2, 0),
                       pred_msg->value_(2, 1),
                       pred_msg->value_(2, 2)).getRotation(q);

        tf2::convert(q, map_sensor_tf.transform.rotation);
        tf_broadcaster_->sendTransform(map_sensor_tf);

        // Only for visualization
        predicted_x_ = map_sensor_tf.transform.translation.x;
        predicted_y_ = map_sensor_tf.transform.translation.y;
    }

}

// Subscribe to the active odom topic
void ArtslamLocalizer::odom_callback(const nav_msgs::msg::Odometry &msg) {
    // 1. If the odometry from Encoders is disabled, immediately return
    if(!use_encoders_) return;

    // 2. Otherwise, create a new Odometry custom message and convert from Odometry
    Odometry_MSG::Ptr odom = std::make_shared<Odometry_MSG>();
    odom->header_.timestamp_ = static_cast<int64_t>(msg.header.stamp.sec * 1000000000ull) + msg.header.stamp.nanosec;
    odom->header_.frame_id_ = msg.header.frame_id;

    // 2.1 Correctly assign translation and rotation to the new message
    odom->value_ = EigMatrix4d::Identity();
    odom->value_.block<3,1>(0,3) = EigVector3d(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z);
    EigQuaterniond quat = EigQuaterniond(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    odom->value_.block<3,3>(0,0) = quat.toRotationMatrix();

    // 2.2 Also convert the covariance, if any
    bool is_covariance_pose_set = false;
    for (int i = 0; i < 36; ++i) {
        if (msg.pose.covariance[i] != 0.0) {
            is_covariance_pose_set = true;
            break;
        }
    }
    odom->covariance_ = EigMatrixXd::Identity(6,6);
    if(is_covariance_pose_set) {
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                odom->covariance_.value()(i,j) = msg.pose.covariance[i * 6 + j];
            }
        }
    }

    // 3. Remember to reset the initial odometry in case the robot was already moving
    if(!first_odom) {
        first_odom = std::make_shared<Odometry_MSG>();
        first_odom->value_ = odom->value_;
    }
    odom->value_ = first_odom->value_.inverse() * odom->value_;

    Odometry_MSG::Ptr pred_msg = localizer_.predict(odom);

    if(!use_imu_) {
        if (pred_msg) {
            // Send a tf to represent the motion of the robot
            geometry_msgs::msg::TransformStamped map_sensor_tf;

            map_sensor_tf.header.stamp = msg.header.stamp;
            map_sensor_tf.header.frame_id = global_frame_;
            map_sensor_tf.child_frame_id = "os_sensor_tmp";

            map_sensor_tf.transform.translation.x = pred_msg->value_(0, 3);
            map_sensor_tf.transform.translation.y = pred_msg->value_(1, 3);
            map_sensor_tf.transform.translation.z = pred_msg->value_(2, 3);

            tf2::Quaternion q;
            tf2::Matrix3x3(pred_msg->value_(0, 0),
                           pred_msg->value_(0, 1),
                           pred_msg->value_(0, 2),
                           pred_msg->value_(1, 0),
                           pred_msg->value_(1, 1),
                           pred_msg->value_(1, 2),
                           pred_msg->value_(2, 0),
                           pred_msg->value_(2, 1),
                           pred_msg->value_(2, 2)).getRotation(q);

            tf2::convert(q, map_sensor_tf.transform.rotation);
            tf_broadcaster_->sendTransform(map_sensor_tf);

            // Only for visualization
            predicted_x_ = pred_msg->value_(0, 3);
            predicted_y_ = pred_msg->value_(1, 3);
        }
    }
}

void ArtslamLocalizer::gnss_callback(const sensor_msgs::msg::NavSatFix &msg) {
    // 1. If the GNSS is disabled, immediately return
    if(!use_gnss_) return;

    // 2. Otherwise, create a new GNSS custom message and convert from NavSatFix
    GNSS_MSG::Ptr _gnss_msg = std::make_shared<GNSS_MSG>();
    _gnss_msg->header_.timestamp_ = static_cast<int64_t>(msg.header.stamp.sec * 1000000000ull) + msg.header.stamp.nanosec;
    _gnss_msg->header_.frame_id_ = msg.header.frame_id;
    _gnss_msg->lla_ = EigVector3d(msg.latitude, msg.longitude, msg.altitude);

    // 2.1 Convert Latitude, Longitude and Altitude to UTM
    double northing, easting;
    int zone;
    char band;
    EigVector3d lla = _gnss_msg->lla_.value();
    TypesConverter::LL_to_UTM(lla(0), lla(1), northing, easting, zone, band);
    EigVector3d xyz(easting, northing, lla(2));
    _gnss_msg->utm_ = xyz;

    // 2.2 Also convert the covariance, if any
    if(msg.position_covariance_type != 0) {
        EigMatrix3d cov = EigMatrix3d::Identity();
        cov(0,0) = msg.position_covariance[0];
        cov(0,1) = msg.position_covariance[1];
        cov(0,2) = msg.position_covariance[2];
        cov(1,0) = msg.position_covariance[3];
        cov(1,1) = msg.position_covariance[4];
        cov(1,2) = msg.position_covariance[5];
        cov(2,0) = msg.position_covariance[6];
        cov(2,1) = msg.position_covariance[7];
        cov(2,2) = msg.position_covariance[8];
        _gnss_msg->covariance_ = cov;
    }

    lots::core::types::Odometry_MSG::Ptr pred_msg = localizer_.correct_with_gnss(_gnss_msg);
    if(!use_encoders_ && !use_imu_) {
        if (pred_msg) {
            // Send a tf to represent the motion of the robot
            geometry_msgs::msg::TransformStamped map_sensor_tf;

            map_sensor_tf.header.stamp = msg.header.stamp;
            map_sensor_tf.header.frame_id = global_frame_;
            map_sensor_tf.child_frame_id = "os_sensor_tmp";

            map_sensor_tf.transform.translation.x = pred_msg->value_(0, 3);
            map_sensor_tf.transform.translation.y = pred_msg->value_(1, 3);
            map_sensor_tf.transform.translation.z = pred_msg->value_(2, 3);

            tf2::Quaternion q;
            tf2::Matrix3x3(pred_msg->value_(0, 0),
                           pred_msg->value_(0, 1),
                           pred_msg->value_(0, 2),
                           pred_msg->value_(1, 0),
                           pred_msg->value_(1, 1),
                           pred_msg->value_(1, 2),
                           pred_msg->value_(2, 0),
                           pred_msg->value_(2, 1),
                           pred_msg->value_(2, 2)).getRotation(q);

            tf2::convert(q, map_sensor_tf.transform.rotation);
            tf_broadcaster_->sendTransform(map_sensor_tf);

            // Only for visualization
            predicted_x_ = pred_msg->value_(0, 3);
            predicted_y_ = pred_msg->value_(1, 3);
        }
    }
}

void ArtslamLocalizer::pointcloud_callback(const sensor_msgs::msg::PointCloud2& msg)
{
    // 1. If the use of point clouds is disabled, immediately return, but only after global localization
    // TODO in localizer, make it so that GPS can be used instead
    if(!use_point_clouds_)
        if(pointcloud_msg_seq_ > 0)
            return;

    // 2. Otherwise, create a new PointCloud and convert from PointCloud2
    pcl::PointCloud<Point3I>::Ptr cloud = std::make_shared<pcl::PointCloud<Point3I>>();
    pcl::fromROSMsg(msg, *cloud);
    cloud->header.stamp = msg.header.stamp.sec * 1000000000ull + msg.header.stamp.nanosec;
    cloud->header.seq = pointcloud_msg_seq_;
    cloud->header.frame_id = msg.header.frame_id;

    pointcloud_msg_seq_++;
    lots::core::types::Odometry_MSG::Ptr pred_msg = localizer_.correct_with_cloud(cloud);

    if (pred_msg) {
        geometry_msgs::msg::TransformStamped map_sensor_tf;

        map_sensor_tf.header.stamp = msg.header.stamp;
        map_sensor_tf.header.frame_id = global_frame_;
        map_sensor_tf.child_frame_id = "os_sensor_tmp"; // TODO use sensor_frame_ (parameter) instead

        map_sensor_tf.transform.translation.x = pred_msg->value_(0, 3);
        map_sensor_tf.transform.translation.y = pred_msg->value_(1, 3);
        map_sensor_tf.transform.translation.z = pred_msg->value_(2, 3);

        tf2::Quaternion q;
        tf2::Matrix3x3(pred_msg->value_(0, 0),
                       pred_msg->value_(0, 1),
                       pred_msg->value_(0, 2),
                       pred_msg->value_(1, 0),
                       pred_msg->value_(1, 1),
                       pred_msg->value_(1, 2),
                       pred_msg->value_(2, 0),
                       pred_msg->value_(2, 1),
                       pred_msg->value_(2, 2)).getRotation(q);
        tf2::convert(q, map_sensor_tf.transform.rotation);
        tf_broadcaster_->sendTransform(map_sensor_tf);

        // Only for visualization
        corrected_x_ = map_sensor_tf.transform.translation.x;
        corrected_y_ = map_sensor_tf.transform.translation.y;
    }
}

// publish the estimate of the robot's pose
void ArtslamLocalizer::odom_timer_callback() {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = global_frame_;
    odom_msg.child_frame_id = base_frame_;

    // TODO be sure that in imu_callback() the predicted x and y will be changed into the position of base_link, not of os_sensor
    odom_msg.pose.pose.position.x = predicted_x_;
    odom_msg.pose.pose.position.y = predicted_y_;

    odom_pub_->publish(odom_msg);

    nav_msgs::msg::Odometry odom2_msg;
    odom2_msg.header.stamp = this->get_clock()->now();
    odom2_msg.header.frame_id = global_frame_;
    odom2_msg.child_frame_id = base_frame_;

    // TODO be sure that in imu_callback() the predicted x and y will be changed into the position of base_link, not of os_sensor
    odom2_msg.pose.pose.position.x = corrected_x_;
    odom2_msg.pose.pose.position.y = corrected_y_;

    odom_c_pub_->publish(odom2_msg);
}




// TODO split the class and the main function

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::Node::SharedPtr node = std::make_shared<ArtslamLocalizer>();
    executor.add_node(node);
    executor.spin();
    //rclcpp::spin(std::make_shared<ArtslamLocalizer>());
    rclcpp::shutdown();
    return 0;
}

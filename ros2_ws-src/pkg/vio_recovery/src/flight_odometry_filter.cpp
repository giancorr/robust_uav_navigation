#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <chrono>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class FlightOdometryFilter : public rclcpp::Node {
public:
    FlightOdometryFilter() : Node("flight_odometry_filter") {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        init_transformations();

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ov_msckf/odomimu", qos,
            std::bind(&FlightOdometryFilter::odom_callback, this, std::placeholders::_1));

        px4_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10);
            
        RCLCPP_INFO(this->get_logger(), "Flight Odometry Filter running in PURE PASSTHROUGH mode (with TF odom rotation)");
    }

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_pub_;

    Eigen::Quaterniond q_enu_to_ned_;
    Eigen::Quaterniond q_flu_to_frd_;

    void init_transformations() {
        Eigen::Matrix3d R_enu_to_ned;
        R_enu_to_ned << 0,  1,  0,
                        1,  0,  0,
                        0,  0, -1;
        q_enu_to_ned_ = Eigen::Quaterniond(R_enu_to_ned);

        Eigen::Matrix3d R_flu_to_frd;
        R_flu_to_frd << 1,  0,  0,
                        0, -1,  0,
                        0,  0, -1;
        q_flu_to_frd_ = Eigen::Quaterniond(R_flu_to_frd);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (std::isnan(msg->pose.pose.position.x) || std::isnan(msg->twist.twist.linear.x)) return;

        geometry_msgs::msg::TransformStamped tf_g2o;
        try {
            tf_g2o = tf_buffer_->lookupTransform("odom", msg->header.frame_id, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for TF odom -> %s...", msg->header.frame_id.c_str());
            return;
        }

        geometry_msgs::msg::PoseStamped pose_in, pose_odom;
        pose_in.header = msg->header;
        pose_in.pose   = msg->pose.pose;
        tf2::doTransform(pose_in, pose_odom, tf_g2o);

        // Extract transformed VIO pose
        Eigen::Vector3d pos_enu(pose_odom.pose.position.x, pose_odom.pose.position.y, pose_odom.pose.position.z);
        Eigen::Quaterniond q_enu(pose_odom.pose.orientation.w, pose_odom.pose.orientation.x, pose_odom.pose.orientation.y, pose_odom.pose.orientation.z);

        // Velocity: OpenVINS publishes linear velocity in local frame (FLU).
        Eigen::Vector3d v_flu(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        Eigen::Vector3d omega_flu(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

        publish_to_px4(msg->header.stamp, pos_enu, v_flu, omega_flu, q_enu);
    }

    void publish_to_px4(
        const rclcpp::Time& stamp,
        const Eigen::Vector3d& pos_enu,
        const Eigen::Vector3d& vel_flu,
        const Eigen::Vector3d& omega_flu,
        const Eigen::Quaterniond& q_body_to_enu)
    {
        px4_msgs::msg::VehicleOdometry out;

        out.timestamp        = this->get_clock()->now().nanoseconds() / 1000ULL;
        out.timestamp_sample = stamp.nanoseconds() / 1000ULL;

        // Position: ENU -> NED
        Eigen::Vector3d pos_ned = q_enu_to_ned_ * pos_enu;
        out.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        out.position[0] = pos_ned.x();
        out.position[1] = pos_ned.y();
        out.position[2] = pos_ned.z();

        // Linear Velocity: body FLU -> body FRD (Comportamento Originale)
        out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
        out.velocity[0] = vel_flu.x();
        out.velocity[1] = -vel_flu.y();
        out.velocity[2] = -vel_flu.z();

        // Angular velocity: OpenVINS publishes gyro bias in this field, so we MUST set it to NaN
        // to prevent PX4 from fusing incorrect angular velocity!
        out.angular_velocity[0] = NAN;
        out.angular_velocity[1] = NAN;
        out.angular_velocity[2] = NAN;

        // Orientation: ENU body -> NED body
        Eigen::Quaterniond q_px4_ned = q_enu_to_ned_ * q_body_to_enu * q_flu_to_frd_;
        out.q[0] = q_px4_ned.w();
        out.q[1] = q_px4_ned.x();
        out.q[2] = q_px4_ned.y();
        out.q[3] = q_px4_ned.z();


        px4_odom_pub_->publish(out);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlightOdometryFilter>());
    rclcpp::shutdown();
    return 0;
}
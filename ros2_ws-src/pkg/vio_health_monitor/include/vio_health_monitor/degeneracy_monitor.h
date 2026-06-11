#ifndef VIO_HEALTH_MONITOR_DEGENERACY_MONITOR_HPP_
#define VIO_HEALTH_MONITOR_DEGENERACY_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>

class DegeneracyMonitor : public rclcpp::Node {
public:
    DegeneracyMonitor();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr eigen_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;

    double k1_, k2_;
    bool is_consistent_;
};

#endif  // VIO_HEALTH_MONITOR_DEGENERACY_MONITOR_HPP_

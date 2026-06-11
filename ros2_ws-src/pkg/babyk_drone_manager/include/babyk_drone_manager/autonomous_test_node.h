#ifndef AUTONOMOUS_TEST_NODE_H
#define AUTONOMOUS_TEST_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <random>
#include <vector>
#include <chrono>

class AutonomousTestNode : public rclcpp::Node
{
public:
    AutonomousTestNode();

private:
    // Timer callback for sending commands
    void command_timer_callback();
    
    // Callback for status updates
    void status_callback(const std_msgs::msg::String::SharedPtr msg);
    
    // Helper functions
    void send_command(const std::string& command);
    void send_takeoff();
    void send_land();
    void send_random_flyto();
    bool is_system_idle();
    
    // ROS2 interfaces
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscriber_;
    rclcpp::TimerBase::SharedPtr command_timer_;
    
    // State variables
    std::string current_status_;
    bool last_command_was_land_;
    bool system_initialized_;
    std::vector<std::string> available_goals_;
    int consecutive_failures_;         // Counter for consecutive failures
    std::string last_command_sent_;    // Track what command was sent
    
    // Random number generation
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_int_distribution<int> command_dist_;
    
    // Parameters
    int command_interval_min_;
    int command_interval_max_;
    int max_wait_time_;
    double land_probability_;
    int max_consecutive_failures_;    // Max failures before emergency land
};

#endif // AUTONOMOUS_TEST_NODE_H
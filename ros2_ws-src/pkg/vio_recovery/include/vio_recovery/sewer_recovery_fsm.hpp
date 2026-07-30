#ifndef SEWER_RECOVERY_FSM_HPP
#define SEWER_RECOVERY_FSM_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

enum class SewerState {
    NAVIGATE,       // Normal autonomous navigation
    HOVER,          // VIO lost: stop move_manager, stabilize
    ASCEND          // Timeout reached: emergency climb
};

class SewerRecoveryFSM : public rclcpp::Node {
public:
    SewerRecoveryFSM();

private:
    void fsm_loop();
    void health_callback(const std_msgs::msg::String::SharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void send_velocity(double vx, double vy, double vz, double omega_z = 0.0);
    std::string state_to_string(SewerState state);

    // FSM State
    SewerState current_state_;

    // Recovery timing
    rclcpp::Time state_entry_time_;

    // Flags for one-shot actions
    bool stop_command_sent_;

    // Position at recovery trigger
    double recovery_start_x_;
    double recovery_start_y_;
    double recovery_start_z_;
    double recovery_start_yaw_;

    // Current odometry
    double current_x_;
    double current_y_;
    double current_z_;
    double current_yaw_;
    bool odometry_received_;

    // FSM arming state 
    bool armed_;
    int consecutive_consistent_count_;
    int min_consistent_to_arm_;

    // Configurable Parameters
    bool enable_recovery_;          // If false, ignore VIO failures
    bool trigger_on_potentially_inconsistent_; // If true, trigger on POTENTIALLY_INCONSISTENT
    double hover_timeout_;          // max seconds to hover before ascending
    double ascend_velocity_;        // m/s vertical velocity for ascending
    rclcpp::Time node_start_time_;  
    rclcpp::Time last_fsm_time_;    

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_health_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    // Publications
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_move_manager_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_ff_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_fsm_state_num_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_teleop_active_;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif

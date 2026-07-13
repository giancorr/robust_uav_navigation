#ifndef VIO_RECOVERY_FSM_HPP
#define VIO_RECOVERY_FSM_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

enum class DroneState {
    NAVIGATE,       // Normal autonomous navigation
    STOP,           // VIO lost: stop move_manager, stabilize
    STRAFE,         // Move laterally towards the wall
    SETTLE,         // Wait a moment after hitting the wall to stabilize
    SWIPE,          // Spawn stain on wall and move forward to swipe it
    RETURN          // Return to corridor center via move_manager
};

class VioRecoveryFSM : public rclcpp::Node {
public:
    VioRecoveryFSM();

private:
    void fsm_loop();
    void health_callback(const std_msgs::msg::String::SharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    void heuristic_callback(const std_msgs::msg::String::SharedPtr msg);
    void send_velocity(double vx, double vy, double vz, double omega_z = 0.0);
    std::string state_to_string(DroneState state);

    // FSM State
    DroneState current_state_;

    // Recovery timing
    rclcpp::Time state_entry_time_;

    // Flags for one-shot actions
    bool stop_command_sent_;
    bool drop_command_sent_;     // ground marker
    bool swipe_command_sent_;    // lateral wall stain
    bool return_command_sent_;

    // Impact detection
    bool impact_detected_;
    bool bad_impact_{false};  // true if impact angle was too oblique for safe swipe

    // Position at recovery trigger (to return to center)
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

    // Trajectory interpolator's internal commanded yaw (for body-frame sync)
    double interp_cmd_yaw_{0.0};
    bool interp_cmd_yaw_received_{false};

    // Direction chosen by heuristic
    std::string strafe_direction_; // "LEFT" or "RIGHT"
    bool direction_received_;
    
    // Swipe direction derived from impact force
    double swipe_vx_body_;
    double swipe_vy_body_;
    double swipe_omega_z_{0.0};
    double swipe_length_;
    double target_yaw_{0.0};  // desired yaw to align parallel to wall
    double strafe_duration_actual_{0.0};

    // Configurable Parameters
    bool enable_recovery_;          // If false, ignore VIO failures
    double delay_before_drop_;      // wait before dropping ground marker (STOP state)
    double strafe_velocity_;        // m/s lateral velocity
    double strafe_timeout_;         // max seconds to strafe before giving up
    double swipe_duration_;         // seconds to push forward during swipe
    double swipe_velocity_;         // m/s forward velocity during swipe
    double impact_force_threshold_; // N to detect wall contact

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_health_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_heuristic_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_interp_cmd_yaw_;

    // Publications
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_move_manager_cmd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_drone_cmd_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_drop_cmd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_swipe_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_fsm_state_num_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
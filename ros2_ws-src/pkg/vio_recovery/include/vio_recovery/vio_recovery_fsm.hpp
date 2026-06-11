#ifndef VIO_RECOVERY_FSM_HPP
#define VIO_RECOVERY_FSM_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <string>

enum class DroneState {
    TAKEOFF,
    NAVIGATING,
    DROP,
    STRAFE,
    SWIPE,
    RECOVER,
    LANDING
};

class VioRecoveryFSM : public rclcpp::Node {
public:
    VioRecoveryFSM();

private:
    void fsm_loop();
    void send_movement(double x, double y, double z, double yaw, double duration);
    std::string state_to_string(DroneState state);

    // Kinematic functions
    double calculate_trajectory_duration(double distance, double v_max, double a_max);
    double calculate_rotation_duration(double angle_rad, double omega_max, double alpha_max);

    // States
    DroneState current_state_;
    bool mission_started_;
    bool initialized_ = false;
    bool odom_received_ = false;

    // Odometry
    double current_x_, current_y_, current_z_, current_yaw_;
    
    // Maneuver Management
    double takeoff_x_, takeoff_y_, takeoff_yaw_target_;
    int nav_step_;
    double current_maneuver_time_;
    rclcpp::Time takeoff_time_, maneuver_start_time_, start_time_;

    // Emergency Variables
    double hold_x_, hold_y_, hold_z_, hold_yaw_;
    double impact_x_, impact_y_;
    double strafe_direction_;
    bool impact_detected_;
    std::string emergency_cmd_;

    // Configurable Parameters
    bool is_sim_;
    bool enable_spray_;
    double cruise_speed_;
    double acceleration_;
    double omega_max_;
    double alpha_max_;
    double impact_threshold_;
    double swipe_length_;

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_decision_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_external_wrench_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;

    // Publications
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_move_cmd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_swipe_cmd_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_drop_cmd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
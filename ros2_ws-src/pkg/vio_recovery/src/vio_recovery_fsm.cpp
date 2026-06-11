#include "vio_recovery/vio_recovery_fsm.hpp"
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <cmath>
#include <algorithm>

VioRecoveryFSM::VioRecoveryFSM() : Node("vio_recovery_fsm_node"),

    // Params, Subscribers and Publishers
    current_state_(DroneState::TAKEOFF), mission_started_(false),
    takeoff_x_(0.0), takeoff_y_(0.0), nav_step_(0), current_maneuver_time_(0.0),
    hold_x_(0.0), hold_y_(0.0), hold_z_(0.0), hold_yaw_(0.0),
    impact_x_(0.0), impact_y_(0.0),
    strafe_direction_(1.0), impact_detected_(false), emergency_cmd_("NONE"), initialized_(false) {

    this->declare_parameter<double>("target_x", 0.0);
    this->declare_parameter<double>("target_y", 28.0);
    this->declare_parameter<double>("target_z", 2.0);
    this->declare_parameter<double>("target_yaw", 1.57); 
    this->declare_parameter<bool>("enable_spray", true);
    this->declare_parameter<double>("cruise_speed", 1.0);
    this->declare_parameter<double>("acceleration", 0.5);
    this->declare_parameter<double>("omega_max", 0.5);
    this->declare_parameter<double>("alpha_max", 0.2);
    this->declare_parameter<double>("impact_threshold", 2.0);
    this->declare_parameter<double>("swipe_length", 1.5);

    enable_spray_ = this->get_parameter("enable_spray").as_bool();
    cruise_speed_ = this->get_parameter("cruise_speed").as_double();
    acceleration_ = this->get_parameter("acceleration").as_double();
    omega_max_ = this->get_parameter("omega_max").as_double();
    alpha_max_ = this->get_parameter("alpha_max").as_double();
    impact_threshold_ = this->get_parameter("impact_threshold").as_double();
    swipe_length_ = this->get_parameter("swipe_length").as_double();

    sub_decision_ = this->create_subscription<std_msgs::msg::String>(
        "/decision/spray_target", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            if (!enable_spray_) return; 
            
            if (current_state_ == DroneState::NAVIGATING && msg->data != "NONE") {
                emergency_cmd_ = msg->data;
                RCLCPP_WARN(this->get_logger(), "EMERGENCY COMMAND RECEIVED: %s", emergency_cmd_.c_str());
            }
        });

    sub_external_wrench_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/drone/external_wrench", 10,
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            if (std::abs(msg->wrench.force.y) > impact_threshold_) {
                impact_detected_ = true;
            }
        });

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    sub_odom_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos,
        [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            current_x_ = msg->position[0]; current_y_ = msg->position[1]; current_z_ = msg->position[2];
            double qw = msg->q[0], qx = msg->q[1], qy = msg->q[2], qz = msg->q[3];
            current_yaw_ = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
            odom_received_ = true; 
        });

    pub_move_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/fsm/trajectory_command", 10);
    
    pub_swipe_cmd_ = this->create_publisher<std_msgs::msg::String>("/command/swipe_paint", 10);
    pub_drop_cmd_ = this->create_publisher<std_msgs::msg::Bool>("/command/drop_marker", 10);
    
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/fsm/current_state", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&VioRecoveryFSM::fsm_loop, this));
    start_time_ = this->now();
}

// Utility functions

double VioRecoveryFSM::calculate_trajectory_duration(double distance, double v_max, double a_max) {
    if (distance <= 0.05) return 1.0;
    double dist_to_accel_decel = (v_max * v_max) / a_max;
    if (distance > dist_to_accel_decel) {
        double t_accel = v_max / a_max;
        double dist_cruise = distance - dist_to_accel_decel;
        double t_cruise = dist_cruise / v_max;
        return (2.0 * t_accel) + t_cruise;
    } else {
        return 2.0 * std::sqrt(distance / a_max);
    }
}

double VioRecoveryFSM::calculate_rotation_duration(double angle_rad, double omega_max, double alpha_max) {
    double angle = std::abs(angle_rad);
    if (angle < 0.01) return 1.0;
    double angle_to_accel_decel = (omega_max * omega_max) / alpha_max;
    if (angle > angle_to_accel_decel) {
        double t_accel = omega_max / alpha_max;
        double angle_cruise = angle - angle_to_accel_decel;
        double t_cruise = angle_cruise / omega_max;
        return (2.0 * t_accel) + t_cruise;
    } else {
        return 2.0 * std::sqrt(angle / alpha_max);
    }
}

void VioRecoveryFSM::send_movement(double x, double y, double z, double yaw, double duration) {
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = {x, y, z, 0.0, 0.0, yaw, duration};
    pub_move_cmd_->publish(cmd);
}

std::string VioRecoveryFSM::state_to_string(DroneState state) {
    switch (state) {
        case DroneState::TAKEOFF: return "TAKEOFF";
        case DroneState::NAVIGATING: return "NAVIGATING";
        case DroneState::DROP: return "DROP";
        case DroneState::STRAFE: return "STRAFE";
        case DroneState::SWIPE: return "SWIPE";
        case DroneState::RECOVER: return "RECOVER";
        case DroneState::LANDING: return "LANDING";
        default: return "UNKNOWN";
    }
}

// Finite State Machine loop
void VioRecoveryFSM::fsm_loop() {

    // Initial state: wait for odometry data
    if (!odom_received_) {
        start_time_ = this->now();
        return;
    }

    // Wait 4 seconds for sensors to stabilize and state to populate
    if (!initialized_) {
        if ((this->now() - start_time_).seconds() < 4.0) return; 
        
        initialized_ = true;
        takeoff_yaw_target_ = current_yaw_;
        RCLCPP_INFO(this->get_logger(), "Sensors aligned and PX4 ready. INITIATING TAKEOFF!");
    }

    std_msgs::msg::String state_msg;
    state_msg.data = state_to_string(current_state_);
    state_pub_->publish(state_msg);

    switch (current_state_) {
        case DroneState::TAKEOFF:
            {
                // Takeoff
                double target_z_abs = std::abs(this->get_parameter("target_z").as_double());
                double t_z = -target_z_abs; 
                
                if (nav_step_ == 0) { 
                    if (!mission_started_) {
                        double dist_z = std::abs(current_z_ - t_z);
                        double dur = calculate_trajectory_duration(dist_z, cruise_speed_, acceleration_);
                        dur = std::max(dur, 3.0);
                        send_movement(current_x_, current_y_, t_z, current_yaw_, dur);
                        mission_started_ = true;
                        takeoff_time_ = this->now();
                    }
                    if (std::abs(current_z_ - t_z) < 0.2 || (this->now() - takeoff_time_).seconds() > 8.0) {
                        mission_started_ = false; nav_step_ = 0; 
                        takeoff_yaw_target_ = current_yaw_;
                        current_state_ = DroneState::NAVIGATING;
                    }
                }
            }
            break;

        case DroneState::NAVIGATING:
            {
                // Handle emergency commands
                if (emergency_cmd_ != "NONE") {
                    mission_started_ = false; nav_step_ = 0;
                    hold_x_ = current_x_; hold_y_ = current_y_; hold_z_ = current_z_; hold_yaw_ = current_yaw_;
                    
                    if (emergency_cmd_.find("DROP") != std::string::npos) current_state_ = DroneState::DROP;
                    else current_state_ = DroneState::STRAFE;
                    break;
                }

                // Normal mission: move to target
                if (!mission_started_) {
                    double t_x = this->get_parameter("target_x").as_double();
                    double t_y = this->get_parameter("target_y").as_double();
                    double t_z = -std::abs(this->get_parameter("target_z").as_double());
                    
                    double dx = t_x - current_x_;
                    double dy = t_y - current_y_;
                    double dist = std::sqrt(dx*dx + dy*dy);
                    double target_heading = std::atan2(dy, dx); 
                    
                    double yaw_diff = std::atan2(std::sin(target_heading - current_yaw_), std::cos(target_heading - current_yaw_));
                    
                    double dur_trans = calculate_trajectory_duration(dist, cruise_speed_, acceleration_);
                    double dur_rot = calculate_rotation_duration(yaw_diff, omega_max_, alpha_max_);
                    double total_dur = std::max({dur_trans, dur_rot, 2.0});

                    send_movement(t_x, t_y, t_z, target_heading, total_dur);
                    maneuver_start_time_ = this->now();
                    current_maneuver_time_ = total_dur;
                    mission_started_ = true;
                } else {
                    // Check if target is reached or timeout
                    double t_x = this->get_parameter("target_x").as_double();
                    double t_y = this->get_parameter("target_y").as_double();
                    double dx = t_x - current_x_;
                    double dy = t_y - current_y_;
                    double dist = std::sqrt(dx*dx + dy*dy);
                    
                    bool timeout = (this->now() - maneuver_start_time_).seconds() > (current_maneuver_time_ + 2.0);
                    
                    if (dist < 0.5 || timeout) {
                        RCLCPP_INFO(this->get_logger(), "Target reached! Initiating landing sequence.");
                        current_state_ = DroneState::LANDING;
                        mission_started_ = false;
                        nav_step_ = 0;
                    }
                }
            }
            break;

        case DroneState::DROP:
            {
                // Calculate stopping distance and initiate deceleration to hover
                if (nav_step_ == 0) {
                    // Kinematic stopping distance to prevent flying backwards due to overshoot
                    double stopping_distance = (cruise_speed_ * cruise_speed_) / (2.0 * acceleration_);
                    
                    double stop_x = hold_x_ + stopping_distance * std::cos(hold_yaw_);
                    double stop_y = hold_y_ + stopping_distance * std::sin(hold_yaw_);
                    
                    double dur = calculate_trajectory_duration(stopping_distance, cruise_speed_, acceleration_);
                    dur = std::max(dur, 1.5);
                    
                    send_movement(stop_x, stop_y, hold_z_, hold_yaw_, dur); 
                    
                    maneuver_start_time_ = this->now();
                    current_maneuver_time_ = dur;
                    nav_step_ = 1;
                }

                //Wait for the drone to fully stop, then trigger the marker drop
                if (nav_step_ == 1 && (this->now() - maneuver_start_time_).seconds() > current_maneuver_time_) {
                    std_msgs::msg::Bool m; m.data = true;
                    pub_drop_cmd_->publish(m);
                    
                    maneuver_start_time_ = this->now();
                    nav_step_ = 2;
                }

                // Wait 1.5s for the marker physics to settle, reset the spawner, and transition state
                if (nav_step_ == 2 && (this->now() - maneuver_start_time_).seconds() > 1.5) {
                    std_msgs::msg::Bool m; m.data = false;
                    pub_drop_cmd_->publish(m);
                    
                    mission_started_ = false; nav_step_ = 0;
                    
                    if (emergency_cmd_.find("STAMP") != std::string::npos) current_state_ = DroneState::STRAFE;
                    else { emergency_cmd_ = "NONE"; current_state_ = DroneState::NAVIGATING; }
                }
            }
            break;

        case DroneState::STRAFE:
            {
                // Calculate strafe trajectory with coasting distance
                if (nav_step_ == 0) {
                    impact_detected_ = false;
                    strafe_direction_ = (emergency_cmd_.find("LEFT") != std::string::npos) ? 1.0 : -1.0;
                    
                    double coast_dist = (cruise_speed_ * cruise_speed_) / (2.0 * acceleration_);
                    coast_dist *= 1.2; 
                    
                    double strafe_dist = 2.5;
                    double target_strafe_x = hold_x_ + coast_dist * std::cos(hold_yaw_) + (strafe_dist * strafe_direction_) * std::sin(hold_yaw_);
                    double target_strafe_y = hold_y_ + coast_dist * std::sin(hold_yaw_) - (strafe_dist * strafe_direction_) * std::cos(hold_yaw_);
                    
                    double dur = calculate_trajectory_duration(strafe_dist, cruise_speed_ * 0.3, acceleration_ * 0.3);
                    dur = std::max(dur, 4.0);
                    
                    send_movement(target_strafe_x, target_strafe_y, hold_z_, hold_yaw_, dur);
                    
                    maneuver_start_time_ = this->now();
                    current_maneuver_time_ = dur;
                    nav_step_ = 1;
                }

                // Wait for impact or timeout
                if (nav_step_ == 1) {
                    bool timeout = (this->now() - maneuver_start_time_).seconds() > current_maneuver_time_;
                    
                    if (impact_detected_ || timeout) {
                        impact_x_ = current_x_; impact_y_ = current_y_;
                        mission_started_ = false; nav_step_ = 0;
                        
                        if (impact_detected_) {
                            RCLCPP_WARN(this->get_logger(), "WALL IMPACT DETECTED! INITIATING SWIPE MANEUVER.");
                            
                            std_msgs::msg::String m;
                            m.data = (strafe_direction_ > 0) ? "LEFT" : "RIGHT";
                            pub_swipe_cmd_->publish(m);
                            
                            current_state_ = DroneState::SWIPE; 
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "STRAFE TIMEOUT REACHED! Returning to original trajectory.");
                            current_state_ = DroneState::RECOVER; 
                        }
                    }
                }
            }
            break;

        case DroneState::SWIPE:
            {
                // Calculate swipe trajectory and initiate movement
                if (nav_step_ == 0) {
                    double target_swipe_x = impact_x_ + swipe_length_ * std::cos(hold_yaw_);
                    double target_swipe_y = impact_y_ + swipe_length_ * std::sin(hold_yaw_);
                    
                    double relax_dist = 0.05; 
                    target_swipe_x -= (relax_dist * strafe_direction_) * std::sin(hold_yaw_);
                    target_swipe_y += (relax_dist * strafe_direction_) * std::cos(hold_yaw_);
                    
                    double dur = calculate_trajectory_duration(swipe_length_, cruise_speed_ * 0.5, acceleration_); 
                    dur = std::max(dur, 2.0);

                    send_movement(target_swipe_x, target_swipe_y, hold_z_, hold_yaw_, dur);
                    
                    maneuver_start_time_ = this->now();
                    current_maneuver_time_ = dur;
                    nav_step_ = 1;
                }

                // Stop swipe maneuver and transition to recover state
                if (nav_step_ == 1 && (this->now() - maneuver_start_time_).seconds() > current_maneuver_time_) {
                    std_msgs::msg::String m; 
                    m.data = "STOP";
                    pub_swipe_cmd_->publish(m); 
                    
                    mission_started_ = false; nav_step_ = 0;
                    current_state_ = DroneState::RECOVER;
                }
            }
            break;

        case DroneState::RECOVER:
            {
                // Calculate perpendicular re-entry point to the original trajectory
                if (nav_step_ == 0) {
                    double dx_curr = current_x_ - hold_x_;
                    double dy_curr = current_y_ - hold_y_;
                    double p = dx_curr * std::cos(hold_yaw_) + dy_curr * std::sin(hold_yaw_);
                    
                    double recover_x = hold_x_ + p * std::cos(hold_yaw_);
                    double recover_y = hold_y_ + p * std::sin(hold_yaw_);
                    
                    double dx = recover_x - current_x_;
                    double dy = recover_y - current_y_;
                    double dist = std::sqrt(dx*dx + dy*dy);
                    
                    double dur = calculate_trajectory_duration(dist, cruise_speed_ * 0.3, acceleration_ * 0.5);
                    dur = std::max(dur, 2.0);

                    send_movement(recover_x, recover_y, hold_z_, hold_yaw_, dur);
                    maneuver_start_time_ = this->now();
                    current_maneuver_time_ = dur;
                    nav_step_ = 1;
                }

                // Resume normal navigation
                if (nav_step_ == 1 && (this->now() - maneuver_start_time_).seconds() > current_maneuver_time_) {
                    emergency_cmd_ = "NONE"; 
                    mission_started_ = false; nav_step_ = 0;
                    current_state_ = DroneState::NAVIGATING; 
                }
            }
            break;

        case DroneState::LANDING:
            {
                // Initiate slow descent
                if (nav_step_ == 0) {
                    double dist_z = std::abs(current_z_ - 0.0);
                    double dur = calculate_trajectory_duration(dist_z, 0.5, acceleration_);
                    dur = std::max(dur, 2.0);
                    
                    send_movement(current_x_, current_y_, 0.0, current_yaw_, dur);
                    
                    maneuver_start_time_ = this->now();
                    current_maneuver_time_ = dur;
                    nav_step_ = 1;
                }

                // Keep grounded and complete landing
                if (nav_step_ == 1 && (this->now() - maneuver_start_time_).seconds() > current_maneuver_time_) {
                    send_movement(current_x_, current_y_, 0.0, current_yaw_, -1.0); 
                    nav_step_ = 2;
                    RCLCPP_INFO(this->get_logger(), "Landing completed successfully.");
                }
            }
            break;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VioRecoveryFSM>());
    rclcpp::shutdown();
    return 0;
}
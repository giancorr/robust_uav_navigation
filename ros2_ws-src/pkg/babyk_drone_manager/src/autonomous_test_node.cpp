#include "babyk_drone_manager/autonomous_test_node.h"

using namespace std::chrono_literals;

AutonomousTestNode::AutonomousTestNode() 
    : Node("autonomous_test_node"),
      current_status_("UNKNOWN"),
      last_command_was_land_(false),
      system_initialized_(false),
      consecutive_failures_(0),
      last_command_sent_(""),
      gen_(rd_()),
      command_dist_(0, 99)
{
    // Declare and get parameters
    this->declare_parameter("command_interval_min", 120);  // 2 minutes
    this->declare_parameter("command_interval_max", 180);  // 3 minutes
    this->declare_parameter("max_wait_time", 60);          // 1 minute max wait
    this->declare_parameter("land_probability", 0.20);     // 20% chance of landing
    this->declare_parameter("max_consecutive_failures", 3); // Max failures before emergency land
    
    command_interval_min_ = this->get_parameter("command_interval_min").as_int();
    command_interval_max_ = this->get_parameter("command_interval_max").as_int();
    max_wait_time_ = this->get_parameter("max_wait_time").as_int();
    land_probability_ = this->get_parameter("land_probability").as_double();
    max_consecutive_failures_ = this->get_parameter("max_consecutive_failures").as_int();
    
    // Available goals (goal1 to goal7)
    available_goals_ = {"goal1", "goal2", "goal3", "goal4", "goal5", "goal6", "goal7"};

    // Create publisher and subscriber
    command_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/seed_pdt_drone/command", 10);
    
    status_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/trajectory_interpolator/status", 10,
        std::bind(&AutonomousTestNode::status_callback, this, std::placeholders::_1));
    
    // Create timer for sending commands
    command_timer_ = this->create_wall_timer(
        5s, std::bind(&AutonomousTestNode::command_timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Autonomous Test Node initialized");
    RCLCPP_INFO(this->get_logger(), "Available goals: goal1-goal7");
    RCLCPP_INFO(this->get_logger(), "Command interval: %d-%d seconds", 
                command_interval_min_, command_interval_max_);
    RCLCPP_INFO(this->get_logger(), "Land probability: %.1f%%", land_probability_ * 100);
    RCLCPP_INFO(this->get_logger(), "Max consecutive failures before emergency land: %d", max_consecutive_failures_);
}

void AutonomousTestNode::status_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string previous_status = current_status_;
    current_status_ = msg->data;
    
    // Track failures: if status changed and we detect a failure condition
    if (previous_status != current_status_) {
        // Check if this represents a failure (you can customize these conditions)
        if (current_status_ == "IDLE" && !last_command_sent_.empty()) {
            // Command completed successfully - reset failure counter
            if (consecutive_failures_ > 0) {
                RCLCPP_INFO(this->get_logger(), "Command '%s' completed successfully. Resetting failure counter.", 
                           last_command_sent_.c_str());
                consecutive_failures_ = 0;
            }
        }
        else if (current_status_.find("ERROR") != std::string::npos || 
                 current_status_.find("FAILED") != std::string::npos) {
            // Command failed - increment failure counter
            consecutive_failures_++;
            RCLCPP_WARN(this->get_logger(), "Command '%s' failed with status '%s'. Consecutive failures: %d/%d", 
                       last_command_sent_.c_str(), current_status_.c_str(), 
                       consecutive_failures_, max_consecutive_failures_);
        }
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Status: %s", current_status_.c_str());
}

void AutonomousTestNode::command_timer_callback()
{
    static auto last_command_time = this->now();
    static auto last_status_change = this->now();
    
    auto current_time = this->now();
    
    // Initial takeoff after system starts
    if (!system_initialized_) {
        if (is_system_idle()) {
            RCLCPP_INFO(this->get_logger(), "System ready, sending initial takeoff");
            send_takeoff();
            system_initialized_ = true;
            last_command_time = current_time;
            last_status_change = current_time;
            return;
        }
        return;
    }
    
    // Update last status change time if status changed
    static std::string previous_status = current_status_;
    if (current_status_ != previous_status) {
        last_status_change = current_time;
        previous_status = current_status_;
    }
    
    // Calculate time intervals
    auto time_since_last_command = (current_time - last_command_time).seconds();
    auto time_since_status_change = (current_time - last_status_change).seconds();
    
    // Generate random interval for next command
    std::uniform_int_distribution<int> interval_dist(command_interval_min_, command_interval_max_);
    int target_interval = interval_dist(gen_);
    
    // Check if we should send a new command
    bool should_send_command = false;
    
    if (is_system_idle()) {
        // If system just became idle, send command immediately
        // Otherwise wait for the interval
        if (time_since_last_command >= 10.0) {  // Minimum 10 seconds between commands
            should_send_command = true;
        }
    }
    else if (time_since_status_change >= max_wait_time_ && time_since_last_command >= target_interval) {
        // System is stuck, force a new command
        should_send_command = true;
        RCLCPP_WARN(this->get_logger(), "Forcing new command: system stuck in '%s' for %.1f seconds", 
                    current_status_.c_str(), time_since_status_change);
    }
    
    if (should_send_command) {
        // EMERGENCY LAND: Too many consecutive failures
        if (consecutive_failures_ >= max_consecutive_failures_ && !last_command_was_land_ && last_command_sent_ != "land") {
            RCLCPP_ERROR(this->get_logger(), "🚨 EMERGENCY LAND: %d consecutive failures detected! Forcing emergency land command.", 
                         consecutive_failures_);
            send_land();
            last_command_was_land_ = true;
            consecutive_failures_ = 0; // Reset counter after emergency land
            RCLCPP_ERROR(this->get_logger(), "🚨 Emergency land sent - next command will MANDATORY be takeoff");
        }
        // If last command was land, next MUST be takeoff (CRITICAL RULE)
        else if (last_command_was_land_ || last_command_sent_ == "land") {
            send_takeoff();
            last_command_was_land_ = false;
            RCLCPP_INFO(this->get_logger(), "MANDATORY takeoff after land command");
        }
        else {
            // Random choice between flyto and land
            double rand_val = static_cast<double>(command_dist_(gen_)) / 100.0;
            
            // Never allow land if last command was already land (double safety check)
            if (last_command_sent_ == "land") {
                RCLCPP_WARN(this->get_logger(), "Preventing land repetition - forcing flyto");
                send_random_flyto();
            }
            // Random choice based on probability
            else if (rand_val >= land_probability_) {
                send_random_flyto();
            }
            else {
                // Send land command
                send_land();
                last_command_was_land_ = true;
                RCLCPP_INFO(this->get_logger(), "Land command sent - next command will be takeoff");
            }
        }
        
        last_command_time = current_time;
        last_status_change = current_time;
    }
}

void AutonomousTestNode::send_command(const std::string& command)
{
    auto msg = std_msgs::msg::String();
    msg.data = command;
    command_publisher_->publish(msg);
    
    // Smart logging with context-aware repetition detection
    if (!last_command_sent_.empty() && command != last_command_sent_) {
        RCLCPP_INFO(this->get_logger(), "✅ Sent command: '%s' (changed from '%s')", 
                    command.c_str(), last_command_sent_.c_str());
    } 
    else if (!last_command_sent_.empty() && command == last_command_sent_) {
        // Check if this is a legitimate repetition
        bool is_legitimate_repeat = false;
        std::string reason = "";
        
        if (command == "takeoff") {
            is_legitimate_repeat = true;
            reason = "retry after failure or after land";
        }
        else if (command == "land") {
            is_legitimate_repeat = false; // Land should never be repeated
            reason = "INVALID - land should not repeat";
        }
        else if (command.substr(0, 5) == "flyto") {
            is_legitimate_repeat = true; // Could be retry after failure
            reason = "retry after failure";
        }
        
        if (is_legitimate_repeat) {
            RCLCPP_INFO(this->get_logger(), "🔄 Sent command: '%s' (repeated - %s)", 
                        command.c_str(), reason.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ Sent command: '%s' (INVALID REPETITION - %s)", 
                        command.c_str(), reason.c_str());
        }
    } 
    else {
        RCLCPP_INFO(this->get_logger(), "🚀 Sent command: '%s' (first command)", command.c_str());
    }
    
    // Update the last command tracking
    last_command_sent_ = command;
}

void AutonomousTestNode::send_takeoff()
{
    send_command("takeoff");
}

void AutonomousTestNode::send_land()
{
    send_command("land");
}

void AutonomousTestNode::send_random_flyto()
{
    std::string new_command;
    int max_attempts = 20; // Safety limit to avoid infinite loops
    int attempts = 0;

    do {
        // Pick a random goal from goal1 to goal7
        std::uniform_int_distribution<int> goal_dist(0, available_goals_.size() - 1);
        int goal_index = goal_dist(gen_);

        // Probabilità 10% per cover/circle, 90% flyto normale
        double special_prob = static_cast<double>(command_dist_(gen_)) / 100.0;
        if (special_prob < 0.05) {
            // new_command = "flyto(cover(" + available_goals_[goal_index] + ",2.0,5.0))";
            new_command = "cover((1,1),(1,4),(4,4),(4,1))"; // new standard for area covering
        } else if (special_prob < 0.10) {
            new_command = "flyto(circle(" + available_goals_[goal_index] + "))";
        } else {
            new_command = "flyto(" + available_goals_[goal_index] + ")";
        }

        attempts++;

        if (attempts >= max_attempts) {
            RCLCPP_WARN(this->get_logger(), "Could not find different goal after %d attempts, allowing repetition", max_attempts);
            break;
        }

    } while (new_command == last_command_sent_ && available_goals_.size() > 1);

    send_command(new_command);

    if (attempts > 1) {
        RCLCPP_INFO(this->get_logger(), "Avoided repeating command, selected different goal after %d attempts", attempts);
    }
}

bool AutonomousTestNode::is_system_idle()
{
    return current_status_ == "IDLE" || 
           current_status_ == "STOPPED" || 
           current_status_ == "MISSION_COMPLETED";
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<AutonomousTestNode>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Autonomous Test Node");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
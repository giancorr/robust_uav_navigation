#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>

class DegeneracyMonitor : public rclcpp::Node {
public:
    //VIO State
    enum class VioState {
        CONSISTENT,
        POTENTIALLY_INCONSISTENT,
        INCONSISTENT,
        POTENTIALLY_CONSISTENT
    };

    DegeneracyMonitor() : Node("degeneracy_monitor"), current_state_(VioState::CONSISTENT) {
        
        // Params, Subscribers and Publishers
        this->declare_parameter<double>("kx1", 40.0);
        this->declare_parameter<double>("ky1", 40.0);
        this->declare_parameter<double>("kz1", 70.0);
        this->declare_parameter<double>("kx2", 60.0);
        this->declare_parameter<double>("ky2", 60.0);
        this->declare_parameter<double>("kz2", 90.0);
        this->declare_parameter<double>("delta_t", 0.4);
        
        this->declare_parameter<double>("stagnation_epsilon", 1e-6);
        this->declare_parameter<int>("max_stagnation_counts", 5);

        this->get_parameter("kx1", kx1_);
        this->get_parameter("ky1", ky1_);
        this->get_parameter("kz1", kz1_);
        this->get_parameter("kx2", kx2_);
        this->get_parameter("ky2", ky2_);
        this->get_parameter("kz2", kz2_);
        this->get_parameter("delta_t", delta_t_);
        this->get_parameter("stagnation_epsilon", eps_);
        this->get_parameter("max_stagnation_counts", max_stagnation_);

        sub_degen_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/ov_msckf/degen_factor", 10,
            std::bind(&DegeneracyMonitor::degen_callback, this, std::placeholders::_1));

        pub_health_ = this->create_publisher<std_msgs::msg::String>("/vio_health_status", 10);
        
        last_x_ = 0.0; 
        last_y_ = 0.0; 
        last_z_ = 0.0;
        stagnation_counter_ = 0;
    }

private:

    void degen_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 6) return;

        // Get from message eigenvalues of J, the information matrix. Store information about Cartesian coordinates
        float lambda_x = msg->data[3];
        float lambda_y = msg->data[4];
        float lambda_z = msg->data[5];
        auto now = this->now();

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                             "Eigenvalues -> X: %.2f | Y: %.2f | Z: %.2f", 
                             lambda_x, lambda_y, lambda_z);

        // If eigenvalues are stagnant, no information is added, thus the drone is lost
        bool is_stagnant = (std::abs(lambda_x - last_x_) < eps_) && 
                           (std::abs(lambda_y - last_y_) < eps_) && 
                           (std::abs(lambda_z - last_z_) < eps_);

        if (is_stagnant && lambda_x > 0.1) { 
            stagnation_counter_++;
        } else {
            stagnation_counter_ = 0;
        }

        last_x_ = lambda_x; 
        last_y_ = lambda_y; 
        last_z_ = lambda_z;

        // If eigenvaues are low or stagnant, VIO is degenerated
        bool is_degenerated = (lambda_x < kx1_) || (lambda_y < ky1_) || (lambda_z < kz1_) || 
                              (stagnation_counter_ >= max_stagnation_);
        
        bool is_recovered = (lambda_x > kx2_) && (lambda_y > ky2_) && (lambda_z > kz2_) && !is_stagnant;

        // Switch state basing on degeneration
        switch (current_state_) {
            case VioState::CONSISTENT:
                if (is_degenerated) {
                    current_state_ = VioState::POTENTIALLY_INCONSISTENT;
                    state_change_start_time_ = now;
                }
                break;

            case VioState::POTENTIALLY_INCONSISTENT:
                if (!is_degenerated) {
                    current_state_ = VioState::CONSISTENT;
                } else if ((now - state_change_start_time_).seconds() >= delta_t_) {
                    current_state_ = VioState::INCONSISTENT;
                }
                break;

            case VioState::INCONSISTENT:
                if (is_recovered) {
                    current_state_ = VioState::POTENTIALLY_CONSISTENT;
                    state_change_start_time_ = now;
                }
                break;

            case VioState::POTENTIALLY_CONSISTENT:
                if (!is_recovered) {
                    current_state_ = VioState::INCONSISTENT;
                } else if ((now - state_change_start_time_).seconds() >= delta_t_) {
                    current_state_ = VioState::CONSISTENT;
                }
                break;
        }

        // Publish consistency message
        std_msgs::msg::String health_msg;
        if (current_state_ == VioState::CONSISTENT) {
            health_msg.data = "CONSISTENT";
        } else if (current_state_ == VioState::POTENTIALLY_INCONSISTENT) {
            health_msg.data = "POTENTIALLY_INCONSISTENT";
        } else if (current_state_ == VioState::POTENTIALLY_CONSISTENT) {
            health_msg.data = "POTENTIALLY_CONSISTENT";
        } else {
            health_msg.data = "INCONSISTENT";
        }

        pub_health_->publish(health_msg);
    }

    double kx1_, ky1_, kz1_;
    double kx2_, ky2_, kz2_;
    double delta_t_, eps_;
    int max_stagnation_, stagnation_counter_;
    float last_x_, last_y_, last_z_;
    
    VioState current_state_;
    rclcpp::Time state_change_start_time_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_degen_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_health_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DegeneracyMonitor>());
    rclcpp::shutdown();
    return 0;
}
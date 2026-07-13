#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <Eigen/Dense>

using std::placeholders::_1;

class WrenchEstimatorNode : public rclcpp::Node
{
public:
    WrenchEstimatorNode() : Node("wrench_estimator_node")
    {
        // Parameters, Subscribers and Publishers
        this->declare_parameter<double>("mass", 2.23); 
        this->declare_parameter<double>("inertia_xx", 0.0420);
        this->declare_parameter<double>("inertia_yy", 0.0280);
        this->declare_parameter<double>("inertia_zz", 0.0600);
        
        this->declare_parameter<double>("k_i_f", 5.0);
        this->declare_parameter<double>("k_i_m", 15.0);
        
        this->declare_parameter<double>("max_thrust_n", 45.0);
        this->declare_parameter<double>("max_torque_nm", 2.5);
        this->declare_parameter<double>("max_force_out", 50.0);
        this->declare_parameter<double>("max_torque_out", 5.0);

        mass_ = this->get_parameter("mass").as_double();
        inertia_ = (Eigen::Matrix3d() << 
            this->get_parameter("inertia_xx").as_double(), 0.0, 0.0,
            0.0, this->get_parameter("inertia_yy").as_double(), 0.0,
            0.0, 0.0, this->get_parameter("inertia_zz").as_double()).finished();
        
        K_I_f_ = Eigen::Matrix3d::Identity() * this->get_parameter("k_i_f").as_double();
        K_I_m_ = Eigen::Matrix3d::Identity() * this->get_parameter("k_i_m").as_double();
        max_thrust_n_ = this->get_parameter("max_thrust_n").as_double();
        max_torque_nm_ = this->get_parameter("max_torque_nm").as_double();
        max_force_out_ = this->get_parameter("max_force_out").as_double();
        max_torque_out_ = this->get_parameter("max_torque_out").as_double();

        wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/drone/external_wrench", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        thrust_sub_ = this->create_subscription<px4_msgs::msg::VehicleThrustSetpoint>(
            "/fmu/out/vehicle_thrust_setpoint", qos,
            std::bind(&WrenchEstimatorNode::thrust_callback, this, _1));

        torque_sub_ = this->create_subscription<px4_msgs::msg::VehicleTorqueSetpoint>(
            "/fmu/out/vehicle_torque_setpoint", qos,
            std::bind(&WrenchEstimatorNode::torque_callback, this, _1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/world/corridor/model/baby_k_0/link/base_link/sensor/imu_sensor/imu", qos,
            std::bind(&WrenchEstimatorNode::imu_callback, this, _1));

    }

private:
    bool is_initialized_ = false;
    rclcpp::Time last_time_;

    Eigen::Vector3d f_e_hat_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_e_hat_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d integral_m_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_omega_0_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d current_thrust_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_torque_ = Eigen::Vector3d::Zero();

    double mass_;
    Eigen::Matrix3d inertia_;
    Eigen::Matrix3d K_I_f_;
    Eigen::Matrix3d K_I_m_;
    double max_thrust_n_;
    double max_torque_nm_;
    double max_force_out_;
    double max_torque_out_;

    // Rotation matrix FRD to FLU
    const Eigen::Matrix3d R_frd_to_flu_ = (Eigen::Matrix3d() << 
         1,  0,  0, 
         0, -1,  0, 
         0,  0, -1).finished();

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    void thrust_callback(const px4_msgs::msg::VehicleThrustSetpoint::SharedPtr msg)
    {
        if (std::isnan(msg->xyz[0]) || std::isnan(msg->xyz[1]) || std::isnan(msg->xyz[2])) return;
        Eigen::Vector3d f_frd(msg->xyz[0], msg->xyz[1], msg->xyz[2]);
        current_thrust_ = R_frd_to_flu_ * f_frd * max_thrust_n_;
    }

    void torque_callback(const px4_msgs::msg::VehicleTorqueSetpoint::SharedPtr msg)
    {
        if (std::isnan(msg->xyz[0]) || std::isnan(msg->xyz[1]) || std::isnan(msg->xyz[2])) return;
        Eigen::Vector3d m_frd(msg->xyz[0], msg->xyz[1], msg->xyz[2]);
        current_torque_ = R_frd_to_flu_ * m_frd * max_torque_nm_;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        rclcpp::Time current_time = msg->header.stamp;

        // NaN check
        if (std::isnan(msg->linear_acceleration.x) || std::isnan(msg->linear_acceleration.y) || std::isnan(msg->linear_acceleration.z) ||
            std::isnan(msg->angular_velocity.x)    || std::isnan(msg->angular_velocity.y)    || std::isnan(msg->angular_velocity.z)) {
            return;
        }

        Eigen::Vector3d a(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        Eigen::Vector3d omega(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

        // Discard physically impossible readings
        if (a.norm() > 500.0 || omega.norm() > 100.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "IMU spike detected (a=%.1f, omega=%.1f), discarded.", a.norm(), omega.norm());
            return;
        }

        if (!is_initialized_) {
            p_omega_0_ = inertia_ * omega; 
            last_time_ = current_time;
            is_initialized_ = true;
            return;
        }

        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt <= 0.0 || dt > 0.1) {
            return; 
        }

        // f_hat_e = integral( K_I_f * (M*a - f - f_hat_e) ) dt
        Eigen::Vector3d f_e_dot = K_I_f_ * ((mass_ * a) - current_thrust_ - f_e_hat_);
        f_e_hat_ += f_e_dot * dt;

        Eigen::Vector3d I_omega = inertia_ * omega;
        Eigen::Vector3d cross_term = I_omega.cross(omega);
        
        // integral_m = integral( m + (I*omega)×omega + m_hat_e ) dt
        Eigen::Vector3d integral_m_dot = current_torque_ + cross_term + m_e_hat_;
        integral_m_ += integral_m_dot * dt;

        // m_hat_e = K_I_m * (I*omega - integral_m - I*omega_0)
        m_e_hat_ = K_I_m_ * (I_omega - integral_m_ - p_omega_0_);

        // Clamp
        f_e_hat_ = f_e_hat_.cwiseMax(-max_force_out_).cwiseMin(max_force_out_);
        m_e_hat_ = m_e_hat_.cwiseMax(-max_torque_out_).cwiseMin(max_torque_out_);

        // NaN/Inf check
        bool f_bad = !std::isfinite(f_e_hat_.x()) || !std::isfinite(f_e_hat_.y()) || !std::isfinite(f_e_hat_.z());
        bool m_bad = !std::isfinite(m_e_hat_.x()) || !std::isfinite(m_e_hat_.y()) || !std::isfinite(m_e_hat_.z());
        if (f_bad || m_bad) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wrench estimator NaN/Inf detected, resetting.");
            f_e_hat_ = Eigen::Vector3d::Zero();
            m_e_hat_ = Eigen::Vector3d::Zero();
            integral_m_ = Eigen::Vector3d::Zero();
            p_omega_0_ = inertia_ * omega;
            return; 
        }

        // Publish wrench
        geometry_msgs::msg::WrenchStamped wrench_msg;
        wrench_msg.header.stamp = current_time;
        wrench_msg.header.frame_id = "base_link"; 

        wrench_msg.wrench.force.x = f_e_hat_.x();
        wrench_msg.wrench.force.y = f_e_hat_.y();
        wrench_msg.wrench.force.z = f_e_hat_.z();

        wrench_msg.wrench.torque.x = m_e_hat_.x();
        wrench_msg.wrench.torque.y = m_e_hat_.y();
        wrench_msg.wrench.torque.z = m_e_hat_.z();

        wrench_pub_->publish(wrench_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WrenchEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
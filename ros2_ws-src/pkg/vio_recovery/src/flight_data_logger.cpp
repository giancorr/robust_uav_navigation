// flight_data_logger.cpp
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>

#include <geometry_msgs/msg/twist.hpp>

struct OdomRecord  { double timestamp, x, y, z, roll, pitch, yaw; };
struct LambdaRecord { double timestamp, lx, ly, lz; };
struct HealthRecord { double timestamp; int state; }; // 0=CONSISTENT, 1=POTENTIALLY_CONSISTENT, 2=POTENTIALLY_INCONSISTENT, 3=INCONSISTENT
struct WrenchRecord { double timestamp, fx, fy, fz, tx, ty, tz; };
struct TwistRecord { double timestamp, vx, vy, vz, wx, wy, wz; };
struct FsmRecord { double timestamp; int state; }; // 0=NAVIGATE, 1=STRAFE, 2=SWIPE, 3=RETURN

class FlightDataLogger : public rclcpp::Node {
public:
    FlightDataLogger() : Node("flight_data_logger") {
        
        start_time_ = this->now();
        auto qos_px4 = rclcpp::SensorDataQoS();

        // PX4 EKF2 Odometry
        sub_px4_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos_px4,
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                OdomRecord rec;
                rec.timestamp = (this->now() - start_time_).seconds();
                rec.x = msg->position[0];
                rec.y = msg->position[1];
                rec.z = msg->position[2];
                double qw = msg->q[0], qx = msg->q[1], qy = msg->q[2], qz = msg->q[3];
                rec.roll  = std::atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy));
                double sinp = 2.0*(qw*qy - qz*qx);
                rec.pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI/2.0, sinp) : std::asin(sinp);
                rec.yaw   = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
                px4_data_.push_back(rec);
            });

        // Our Tactile Odometry (PX4 Input)
        sub_tactile_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10,
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                OdomRecord rec;
                rec.timestamp = (this->now() - start_time_).seconds();
                rec.x = msg->position[0];
                rec.y = msg->position[1];
                rec.z = msg->position[2];
                double qw = msg->q[0], qx = msg->q[1], qy = msg->q[2], qz = msg->q[3];
                rec.roll  = std::atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy));
                double sinp = 2.0*(qw*qy - qz*qx);
                rec.pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI/2.0, sinp) : std::asin(sinp);
                rec.yaw   = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
                tactile_data_.push_back(rec);
            });

        // OpenVINS VIO Odometry
        sub_vio_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ov_msckf/odomimu", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                OdomRecord rec;
                rec.timestamp = (this->now() - start_time_).seconds();
                rec.x = msg->pose.pose.position.x;
                rec.y = msg->pose.pose.position.y;
                rec.z = msg->pose.pose.position.z;
                double qw = msg->pose.pose.orientation.w, qx = msg->pose.pose.orientation.x;
                double qy = msg->pose.pose.orientation.y, qz = msg->pose.pose.orientation.z;
                rec.roll  = std::atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy));
                double sinp = 2.0*(qw*qy - qz*qx);
                rec.pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI/2.0, sinp) : std::asin(sinp);
                rec.yaw   = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
                vio_data_.push_back(rec);
            });

        // Gazebo Ground Truth
        sub_gt_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/baby_k_0/odometry", qos_px4,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                OdomRecord rec;
                rec.timestamp = (this->now() - start_time_).seconds();
                rec.x = msg->pose.pose.position.x;
                rec.y = msg->pose.pose.position.y;
                rec.z = msg->pose.pose.position.z;
                double qw = msg->pose.pose.orientation.w, qx = msg->pose.pose.orientation.x;
                double qy = msg->pose.pose.orientation.y, qz = msg->pose.pose.orientation.z;
                rec.roll  = std::atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy));
                double sinp = 2.0*(qw*qy - qz*qx);
                rec.pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI/2.0, sinp) : std::asin(sinp);
                rec.yaw   = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
                gt_data_.push_back(rec);
            });

        // Degeneracy Eigenvalues
        sub_lambda_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/ov_msckf/degen_factor", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() < 6) return;
                LambdaRecord rec;
                rec.timestamp = (this->now() - start_time_).seconds();
                rec.lx = msg->data[3];
                rec.ly = msg->data[4];
                rec.lz = msg->data[5];
                lambda_data_.push_back(rec);
            });

        // VIO Health Status
        sub_health_ = this->create_subscription<std_msgs::msg::String>(
            "/vio_health_status", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                HealthRecord rec;
                rec.timestamp = (this->now() - start_time_).seconds();
                
                if      (msg->data == "CONSISTENT")               rec.state = 0;
                else if (msg->data == "POTENTIALLY_CONSISTENT")   rec.state = 1;
                else if (msg->data == "POTENTIALLY_INCONSISTENT") rec.state = 2;
                else                                              rec.state = 3;
                
                health_data_.push_back(rec);
            });

        // External Wrench
        sub_wrench_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/drone/external_wrench", 10,
            [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                WrenchRecord rec;
                rec.timestamp = (this->now() - start_time_).seconds();
                rec.fx = msg->wrench.force.x;
                rec.fy = msg->wrench.force.y;
                rec.fz = msg->wrench.force.z;
                rec.tx = msg->wrench.torque.x;
                rec.ty = msg->wrench.torque.y;
                rec.tz = msg->wrench.torque.z;
                wrench_data_.push_back(rec);
            });

        // Teleop Velocity Command
        sub_teleop_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/teleop/velocity_increments", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                TwistRecord rec;
                rec.timestamp = (this->now() - start_time_).seconds();
                rec.vx = msg->linear.x;
                rec.vy = msg->linear.y;
                rec.vz = msg->linear.z;
                rec.wx = msg->angular.x;
                rec.wy = msg->angular.y;
                rec.wz = msg->angular.z;
                teleop_data_.push_back(rec);
            });

        // FSM State
        sub_fsm_ = this->create_subscription<std_msgs::msg::String>(
            "/fsm/current_state", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                FsmRecord rec;
                rec.timestamp = (this->now() - start_time_).seconds();
                if      (msg->data == "NAVIGATE") rec.state = 0;
                else if (msg->data == "STRAFE")   rec.state = 1;
                else if (msg->data == "SWIPE")    rec.state = 2;
                else if (msg->data == "RETURN")   rec.state = 3;
                else                              rec.state = -1;
                fsm_data_.push_back(rec);
            });

        px4_data_.reserve(30000);
        tactile_data_.reserve(30000);
        vio_data_.reserve(30000);
        gt_data_.reserve(30000);
        lambda_data_.reserve(30000);
        health_data_.reserve(10000);
        wrench_data_.reserve(30000);
        teleop_data_.reserve(10000);
        fsm_data_.reserve(1000);
    }

    void save_data() {
        RCLCPP_INFO(this->get_logger(), "Saving log files...");

        auto save_odom = [](const std::string& filename, const std::vector<OdomRecord>& data) {
            std::ofstream f(filename);
            f << "Time X Y Z Roll Pitch Yaw\n";
            for (const auto& r : data)
                f << r.timestamp << " " << r.x << " " << r.y << " " << r.z
                  << " " << r.roll << " " << r.pitch << " " << r.yaw << "\n";
        };

        save_odom("log_px4.txt",          px4_data_);
        save_odom("log_tactile.txt",      tactile_data_);
        save_odom("log_vio.txt",          vio_data_);
        save_odom("log_ground_truth.txt", gt_data_);

        {
            std::ofstream f("log_eigenvalues.txt");
            f << "Time LX LY LZ\n";
            for (const auto& r : lambda_data_)
                f << r.timestamp << " " << r.lx << " " << r.ly << " " << r.lz << "\n";
        }

        {
            std::ofstream f("log_health.txt");
            f << "Time State\n";
            for (const auto& r : health_data_)
                f << r.timestamp << " " << r.state << "\n";
        }

        {
            std::ofstream f("log_wrench.txt");
            f << "Time Fx Fy Fz Tx Ty Tz\n";
            for (const auto& r : wrench_data_)
                f << r.timestamp << " " << r.fx << " " << r.fy << " " << r.fz
                  << " " << r.tx << " " << r.ty << " " << r.tz << "\n";
        }

        {
            std::ofstream f("log_teleop.txt");
            f << "Time Vx Vy Vz Wx Wy Wz\n";
            for (const auto& r : teleop_data_)
                f << r.timestamp << " " << r.vx << " " << r.vy << " " << r.vz
                  << " " << r.wx << " " << r.wy << " " << r.wz << "\n";
        }

        {
            std::ofstream f("log_fsm.txt");
            f << "Time State\n";
            for (const auto& r : fsm_data_)
                f << r.timestamp << " " << r.state << "\n";
        }

        RCLCPP_INFO(this->get_logger(), "All logs saved: px4, tactile, vio, ground_truth, eigenvalues, health, wrench, teleop, fsm.");
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_px4_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_tactile_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         sub_vio_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         sub_gt_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_lambda_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            sub_health_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr        sub_teleop_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            sub_fsm_;

    rclcpp::Time start_time_;

    std::vector<OdomRecord>  px4_data_;
    std::vector<OdomRecord>  tactile_data_;
    std::vector<OdomRecord>  vio_data_;
    std::vector<OdomRecord>  gt_data_;
    std::vector<LambdaRecord> lambda_data_;
    std::vector<HealthRecord> health_data_;
    std::vector<WrenchRecord> wrench_data_;
    std::vector<TwistRecord>  teleop_data_;
    std::vector<FsmRecord>    fsm_data_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FlightDataLogger>();
    rclcpp::spin(node);
    node->save_data();
    rclcpp::shutdown();
    return 0;
}
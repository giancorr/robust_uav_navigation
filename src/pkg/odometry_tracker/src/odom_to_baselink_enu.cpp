#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class OdomToBaselinkEnu : public rclcpp::Node
{
public:
    OdomToBaselinkEnu() : Node("odom_to_baselink_enu")
    {
        odom_front_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/front/odomimu", 10,
            std::bind(&OdomToBaselinkEnu::front_callback, this, std::placeholders::_1));

        odom_back_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/back/odomimu", 10,
            std::bind(&OdomToBaselinkEnu::back_callback, this, std::placeholders::_1));

        front_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/front/base_link_odom", 10);
        back_pub_  = this->create_publisher<nav_msgs::msg::Odometry>("/back/base_link_odom", 10);

        // Come richiesto: rotazione di SOLI 180 gradi su Z, poi la traslazione
        tf2::Transform T_base_imu_front;
        T_base_imu_front.setOrigin(tf2::Vector3(0.13, 0.0, 0.165));
        tf2::Quaternion q_front(0.5, 0.5, 0.5, -0.5);
        T_base_imu_front.setRotation(q_front);
        T_imu_front_base_ = T_base_imu_front.inverse();

        // Front camera world transform: pure Yaw rotation (Roll=0, Pitch=0)
        tf2::Quaternion q_ov_ros_front;
        q_ov_ros_front.setRPY(0.0, 0.0, M_PI / 2.0);
        T_ov_to_ros_front_.setRotation(q_ov_ros_front);
        T_ov_to_ros_front_.setOrigin(tf2::Vector3(0, 0, 0));

        // Back T265: derived rigorously from Kalibr calibration
        // T_base_imu1 = T_base_imu0 * inv(T_cam1_imu0) * T_cam0_imu0
        // where T_cam1_imu0 is from multicam kalibr, T_cam0_imu0 is internal T265 geometry
        tf2::Transform T_base_imu_back;
        T_base_imu_back.setOrigin(tf2::Vector3(0.394778, -0.030152, 0.175882));
        T_base_imu_back.setRotation(tf2::Quaternion(-0.35977651, 0.35132169, -0.60757326, 0.61480782));
        T_imu_back_base_ = T_base_imu_back.inverse();

        // Back camera world transform: Rz(+90°) to align OV world (FLU) -> ENU
        // Same as front: FLU -> Rz(+90) -> RFU -> px4_tf_pub -> NED
        tf2::Quaternion q_ov_ros_back;
        q_ov_ros_back.setRPY(0.0, 0.0, M_PI / 2.0);
        T_ov_to_ros_back_.setRotation(q_ov_ros_back);
        T_ov_to_ros_back_.setOrigin(tf2::Vector3(0, 0, 0));

        RCLCPP_INFO(this->get_logger(), "OdomToBaselinkEnu started (Front & Back World Transforms configured).");
    }

private:
    void front_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (first_msg_front_) {
            tf2::Transform T_global_imu;
            tf2::fromMsg(msg->pose.pose, T_global_imu);
            T_init_pos_front_ = (T_ov_to_ros_front_ * T_global_imu * T_imu_front_base_).getOrigin();
            first_msg_front_ = false;
        }
        process_odom(msg, T_imu_front_base_, T_ov_to_ros_front_, front_pub_, T_init_pos_front_);
    }

    void back_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (first_msg_back_) {
            tf2::Transform T_global_imu;
            tf2::fromMsg(msg->pose.pose, T_global_imu);
            T_init_pos_back_ = (T_ov_to_ros_back_ * T_global_imu * T_imu_back_base_).getOrigin();
            first_msg_back_ = false;
        }
        process_odom(msg, T_imu_back_base_, T_ov_to_ros_back_, back_pub_, T_init_pos_back_);
    }

    void process_odom(const nav_msgs::msg::Odometry::SharedPtr msg, 
                      const tf2::Transform& T_imu_base, 
                      const tf2::Transform& T_ov_to_ros,
                      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub,
                      const tf2::Vector3& T_init_pos) 
    {
        tf2::Transform T_global_imu;
        tf2::fromMsg(msg->pose.pose, T_global_imu);

        // Calcolo la posa del base_link ruotando il mondo in modo specifico per la telecamera attiva
        tf2::Transform T_global_base = T_ov_to_ros * T_global_imu * T_imu_base;

        nav_msgs::msg::Odometry out;
        out.header.stamp = msg->header.stamp;
        out.header.frame_id = "global"; // STESSO FRAME DI ODOMIMU!
        out.child_frame_id = "base_link";

        // Azzeriamo SOLO LA POSIZIONE iniziale, così parte da 0,0,0 
        // ma mantiene l'orientamento esatto di OpenVINS.
        tf2::Vector3 pos = T_global_base.getOrigin() - T_init_pos;
        tf2::Quaternion rot = T_global_base.getRotation();

        out.pose.pose.position.x = pos.x();
        out.pose.pose.position.y = pos.y();
        out.pose.pose.position.z = pos.z();
        out.pose.pose.orientation.x = rot.x();
        out.pose.pose.orientation.y = rot.y();
        out.pose.pose.orientation.z = rot.z();
        out.pose.pose.orientation.w = rot.w();

        out.pose.covariance = msg->pose.covariance;

        // Twist (velocità nel frame body)
        tf2::Matrix3x3 R_imu_base(T_imu_base.getRotation());
        tf2::Vector3 v_imu(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        tf2::Vector3 w_imu(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

        tf2::Vector3 v_base = R_imu_base.inverse() * v_imu;
        tf2::Vector3 w_base = R_imu_base.inverse() * w_imu;

        out.twist.twist.linear.x = v_base.x();
        out.twist.twist.linear.y = v_base.y();
        out.twist.twist.linear.z = v_base.z();
        
        out.twist.twist.angular.x = w_base.x();
        out.twist.twist.angular.y = w_base.y();
        out.twist.twist.angular.z = w_base.z();

        out.twist.covariance = msg->twist.covariance;

        pub->publish(out);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_front_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_back_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr front_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr back_pub_;

    tf2::Transform T_imu_front_base_;
    tf2::Transform T_imu_back_base_;
    tf2::Transform T_ov_to_ros_front_;
    tf2::Transform T_ov_to_ros_back_;

    bool first_msg_front_ = true;
    bool first_msg_back_ = true;
    tf2::Vector3 T_init_pos_front_;
    tf2::Vector3 T_init_pos_back_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToBaselinkEnu>());
    rclcpp::shutdown();
    return 0;
}

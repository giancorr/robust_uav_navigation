#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <cmath>

using std::placeholders::_1;

class OdomToBaselinkEnuDirect : public rclcpp::Node
{
public:
    OdomToBaselinkEnuDirect() : Node("odom_to_baselink_enu_direct") {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/back/odomimu", 10, std::bind(&OdomToBaselinkEnuDirect::odom_callback, this, _1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "OdomToBaselinkEnuDirect: FLU→ENU simple converter started.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // --- READ INPUT (FLU) ---
        double fx = msg->pose.pose.position.x;
        double fy = msg->pose.pose.position.y;
        double fz = msg->pose.pose.position.z;

        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        // --- FLU → ENU position: Rz(+90°) ---
        // ENU_x = -FLU_y  (East = -Left = Right)
        // ENU_y =  FLU_x  (North = Forward)
        // ENU_z =  FLU_z  (Up = Up)
        double ex =  -fy;
        double ey =  fx;
        double ez =  fz;

        // --- FLU → ENU orientation: pre-multiply by Rz(+90°) ---
        // q_rz90 = (w=0.7071, x=0, y=0, z=0.7071)
        // q_enu = q_rz90 * q_flu
        static const double cq = 0.70710678118; // cos(45°) = sin(45°)
        double ew = cq * qw - cq * qz;
        double eqx = cq * qx - cq * qy;
        double eqy = cq * qy + cq * qx;
        double eqz = cq * qz + cq * qw;
        // Normalize
        double norm = std::sqrt(ew*ew + eqx*eqx + eqy*eqy + eqz*eqz);
        ew /= norm; eqx /= norm; eqy /= norm; eqz /= norm;

        // --- Zero initial position AND compute orientation correction ---
        if (first_msg_) {
            init_ex_ = ex;
            init_ey_ = ey;
            init_ez_ = ez;

            // Compute empirical body correction: q_corr = q_init^(-1) * Rz(90°)
            // Applied as RIGHT-multiply: q_out = q_enu(t) * q_corr
            // At t=0: q_out = q_init * q_init^(-1) * Rz(90°) = Rz(90°) → NED yaw=0
            double iw = ew, ix = -eqx, iy = -eqy, iz = -eqz; // q_init^(-1)
            // Hamilton multiply: (iw,ix,iy,iz) * (cq, 0, 0, cq) where cq=cos(45°)
            qcw_ = iw*cq - iz*cq;
            qcx_ = ix*cq + iy*cq;
            qcy_ = -ix*cq + iy*cq;
            qcz_ = iw*cq + iz*cq;
            double cn = std::sqrt(qcw_*qcw_ + qcx_*qcx_ + qcy_*qcy_ + qcz_*qcz_);
            qcw_ /= cn; qcx_ /= cn; qcy_ /= cn; qcz_ /= cn;

            first_msg_ = false;
            RCLCPP_INFO(this->get_logger(), "Init ENU q=(%.4f,%.4f,%.4f,%.4f) corr=(%.4f,%.4f,%.4f,%.4f)",
                eqx, eqy, eqz, ew, qcx_, qcy_, qcz_, qcw_);
        }
        ex -= init_ex_;
        ey -= init_ey_;
        ez -= init_ez_;

        // --- Apply orientation: q_out = q_enu * q_correction (RIGHT-multiply) ---
        // This preserves world-frame rotations correctly
        double ow = ew*qcw_ - eqx*qcx_ - eqy*qcy_ - eqz*qcz_;
        double ox = ew*qcx_ + eqx*qcw_ + eqy*qcz_ - eqz*qcy_;
        double oy = ew*qcy_ - eqx*qcz_ + eqy*qcw_ + eqz*qcx_;
        double oz = ew*qcz_ + eqx*qcy_ - eqy*qcx_ + eqz*qcw_;
        double on = std::sqrt(ow*ow + ox*ox + oy*oy + oz*oz);
        ew = ow/on; eqx = ox/on; eqy = oy/on; eqz = oz/on;

        // --- DEBUG: print every 2 seconds ---
        auto now = this->now();
        if ((now - last_print_).seconds() > 2.0) {
            double r, p, y;
            tf2::Quaternion q_debug(eqx, eqy, eqz, ew);
            tf2::Matrix3x3(q_debug).getRPY(r, p, y);
            RCLCPP_INFO(this->get_logger(),
                "IN(FLU) pos=(%.2f,%.2f,%.2f) | OUT(ENU) pos=(%.2f,%.2f,%.2f) RPY=(%.1f,%.1f,%.1f)deg",
                fx, fy, fz, ex, ey, ez,
                r * 180.0 / M_PI, p * 180.0 / M_PI, y * 180.0 / M_PI);
            last_print_ = now;
        }

        // --- PUBLISH ---
        nav_msgs::msg::Odometry out;
        out.header.stamp = msg->header.stamp;
        out.header.frame_id = "odom";
        out.child_frame_id = "base_link";

        out.pose.pose.position.x = ex;
        out.pose.pose.position.y = ey;
        out.pose.pose.position.z = ez;
        out.pose.pose.orientation.x = eqx;
        out.pose.pose.orientation.y = eqy;
        out.pose.pose.orientation.z = eqz;
        out.pose.pose.orientation.w = ew;

        out.pose.covariance = msg->pose.covariance;
        out.twist = msg->twist;

        odom_pub_->publish(out);

        // --- TF: odom → base_link ---
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = msg->header.stamp;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = ex;
        tf.transform.translation.y = ey;
        tf.transform.translation.z = ez;
        tf.transform.rotation.x = eqx;
        tf.transform.rotation.y = eqy;
        tf.transform.rotation.z = eqz;
        tf.transform.rotation.w = ew;
        tf_broadcaster_->sendTransform(tf);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    bool first_msg_ = true;
    double init_ex_ = 0, init_ey_ = 0, init_ez_ = 0;
    double qcw_ = 1, qcx_ = 0, qcy_ = 0, qcz_ = 0;  // body correction quat
    rclcpp::Time last_print_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToBaselinkEnuDirect>());
    rclcpp::shutdown();
    return 0;
}

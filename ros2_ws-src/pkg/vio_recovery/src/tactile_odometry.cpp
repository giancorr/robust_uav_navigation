#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TactileOdometry : public rclcpp::Node {
public:
    TactileOdometry() : Node("tactile_odometry"), is_initialized_(false) {
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        this->declare_parameter<double>("mass", 2.23);
        this->declare_parameter<double>("inertia_xx", 0.0420);
        this->declare_parameter<double>("inertia_yy", 0.0280);
        this->declare_parameter<double>("inertia_zz", 0.0600);
        this->declare_parameter<bool>("enable_tactile_odometry", true);
        this->declare_parameter<double>("impact_force_threshold", 5.0);
        
        mass_ = this->get_parameter("mass").as_double();
        enable_tactile_ = this->get_parameter("enable_tactile_odometry").as_bool();
        contact_threshold_ = this->get_parameter("impact_force_threshold").as_double();
        inertia_ = (Eigen::Matrix3d() << 
            this->get_parameter("inertia_xx").as_double(), 0.0, 0.0,
            0.0, this->get_parameter("inertia_yy").as_double(), 0.0,
            0.0, 0.0, this->get_parameter("inertia_zz").as_double()).finished();
        inertia_inv_ = inertia_.inverse();

        init_transformations();

        wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/drone/external_wrench", 10,
            std::bind(&TactileOdometry::wrench_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ov_msckf/odomimu", 10,
            std::bind(&TactileOdometry::odom_callback, this, std::placeholders::_1));

        px4_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10);
            
        fsm_state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/fsm/current_state", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                current_state_ = msg->data;
            });
    }

private:

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    double mass_;
    bool enable_tactile_ = true;
    Eigen::Matrix3d inertia_;
    Eigen::Matrix3d inertia_inv_;
    bool is_initialized_;
    rclcpp::Time last_time_;
    
    std::string current_state_ = "NAVIGATING";

    // Unilateral Constraint Variables
    Eigen::Vector3d wall_point_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d wall_normal_enu_ = Eigen::Vector3d::UnitY();
    Eigen::Vector3d last_hybrid_pos_ = Eigen::Vector3d::Zero(); 


    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fsm_state_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_pub_;

    // Latest external wrench
    Eigen::Vector3d last_force_body_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_torque_body_ = Eigen::Vector3d::Zero();
    rclcpp::Time last_wrench_time_;
    Eigen::Vector3d force_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_thrust_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d filtered_thrust_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_body_bias_ = Eigen::Vector3d::Zero();
    double max_thrust_n_ = 30.0;

    // State Machine Variables
    bool was_in_contact_ = false;
    Eigen::Vector3d pos_offset_ = Eigen::Vector3d::Zero();
    double contact_threshold_ = 5.0; // Newton

    // Transformations
    Eigen::Quaterniond q_enu_to_ned_;
    Eigen::Quaterniond q_flu_to_frd_;

    void init_transformations() {
        Eigen::Matrix3d R_enu_to_ned;
        
        // Rotation matrices for standard frame transformations
        R_enu_to_ned << 1,  0,  0,
                        0, -1,  0,
                        0,  0, -1;
        q_enu_to_ned_ = Eigen::Quaterniond(R_enu_to_ned);

        Eigen::Matrix3d R_flu_to_frd;
        R_flu_to_frd << 1,  0,  0,
                        0, -1,  0,
                        0,  0, -1;
        q_flu_to_frd_ = Eigen::Quaterniond(R_flu_to_frd);

        this->declare_parameter("max_thrust_n", 30.0);
        max_thrust_n_ = this->get_parameter("max_thrust_n").as_double();

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        thrust_sub_ = this->create_subscription<px4_msgs::msg::VehicleThrustSetpoint>(
            "/fmu/out/vehicle_thrust_setpoint", qos,
            std::bind(&TactileOdometry::thrust_callback, this, std::placeholders::_1));
    }

    void thrust_callback(const px4_msgs::msg::VehicleThrustSetpoint::SharedPtr msg) {
        if (std::isnan(msg->xyz[0]) || std::isnan(msg->xyz[1]) || std::isnan(msg->xyz[2])) return;
        // Rotation from FRD to FLU
        Eigen::Vector3d f_frd(msg->xyz[0], msg->xyz[1], msg->xyz[2]);
        Eigen::Matrix3d R_frd_to_flu;
        R_frd_to_flu << 1,  0,  0,
                        0, -1,  0,
                        0,  0, -1;
        current_thrust_ = R_frd_to_flu * f_frd * max_thrust_n_;
    }

    void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        if (std::isnan(msg->wrench.force.x) || std::isnan(msg->wrench.torque.x)) return;
        last_force_body_ = Eigen::Vector3d(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
        last_torque_body_ = Eigen::Vector3d(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
        last_wrench_time_ = msg->header.stamp;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (std::isnan(msg->pose.pose.position.x) || std::isnan(msg->twist.twist.linear.x)) return;

        rclcpp::Time current_time = msg->header.stamp;
        
        // COORDINATE TRANSFORMATIONS      
        // Transform the pose to 'odom' frame to ensure it is always standard ENU
        geometry_msgs::msg::PoseStamped pose_in;
        pose_in.header = msg->header;
        pose_in.pose = msg->pose.pose;
        geometry_msgs::msg::PoseStamped pose_odom;
        try {
            // Use TimePointZero to avoid deadlock in single threaded executor!
            geometry_msgs::msg::TransformStamped tf_g2o = tf_buffer_->lookupTransform("odom", msg->header.frame_id, tf2::TimePointZero);
            tf2::doTransform(pose_in, pose_odom, tf_g2o);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF error global->odom: %s", ex.what());
            return;
        }
        
        // Transform the velocity vector to 'odom' frame
        geometry_msgs::msg::Vector3Stamped vel_in;
        vel_in.header = msg->header;
        vel_in.vector = msg->twist.twist.linear;
        geometry_msgs::msg::Vector3Stamped vel_odom;
        try {
            geometry_msgs::msg::TransformStamped tf_g2o = tf_buffer_->lookupTransform("odom", msg->header.frame_id, rclcpp::Time(0));
            tf2::doTransform(vel_in, vel_odom, tf_g2o);
        } catch (tf2::TransformException &ex) {
            return;
        }

        // Convert standard ENU (odom) to pseudo-FLU (expected by our internal logic)
        Eigen::Vector3d pos_meas(pose_odom.pose.position.y, -pose_odom.pose.position.x, pose_odom.pose.position.z);
        Eigen::Vector3d vel_meas(vel_odom.vector.y, -vel_odom.vector.x, vel_odom.vector.z);
        
        // Convert orientation from ENU to pseudo-FLU (-90 deg rotation around Z)
        Eigen::Quaterniond q_enu(pose_odom.pose.orientation.w, pose_odom.pose.orientation.x, pose_odom.pose.orientation.y, pose_odom.pose.orientation.z);
        Eigen::Quaterniond q_enu_to_pseudo_flu(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond q_pseudo_flu = q_enu_to_pseudo_flu * q_enu;
        
        Eigen::Vector4d quat_meas(q_pseudo_flu.w(), q_pseudo_flu.x(), q_pseudo_flu.y(), q_pseudo_flu.z());
        Eigen::Vector3d omega_meas(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

        // IMPACT DETECTION     
        if (!is_initialized_) {
            last_time_ = current_time;
            last_hybrid_pos_ = pos_meas;
            is_initialized_ = true;
            return;
        }

        Eigen::Vector3d true_impact_force = last_force_body_ - force_bias_;
        double force_norm = true_impact_force.norm();
        bool is_impacting = enable_tactile_ && (force_norm > contact_threshold_);
        
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        
        Eigen::Vector3d hybrid_pos;
        Eigen::Vector3d hybrid_vel;
        Eigen::Quaterniond q_bypass(quat_meas(0), quat_meas(1), quat_meas(2), quat_meas(3));

        // UNILATERAL KINEMATIC PROJECTION
        
        if (!enable_tactile_) {

            hybrid_pos = pos_meas;
            hybrid_vel = vel_meas;

        } else if (is_impacting) {
            
            // Calculate contact normal vector
            Eigen::Quaterniond q_curr(quat_meas(0), quat_meas(1), quat_meas(2), quat_meas(3));
            Eigen::Vector3d instant_normal = q_curr * (true_impact_force / force_norm);
            instant_normal.z() = 0.0;
            
            if (instant_normal.norm() > 1e-6) {
                instant_normal.normalize();
                if (!was_in_contact_) {
                    wall_normal_enu_ = instant_normal;
                } else {
                    // Low-pass filter the normal for stability
                    wall_normal_enu_ = 0.95 * wall_normal_enu_ + 0.05 * instant_normal;
                    wall_normal_enu_.normalize();
                }
            }

            if (!was_in_contact_) {
                // Transition: Flight -> Contact
                wall_point_ = pos_meas + pos_offset_;
                was_in_contact_ = true;
                RCLCPP_INFO(this->get_logger(), "IMPACT DETECTED! Unilateral Constraint Activated. Force: %.2f N", force_norm);
                RCLCPP_INFO(this->get_logger(), "Wall Normal: (%.2f, %.2f, %.2f)", wall_normal_enu_.x(), wall_normal_enu_.y(), wall_normal_enu_.z());
            }
            
            Eigen::Vector3d current_pos_est = pos_meas + pos_offset_;
            double d_pen = (current_pos_est - wall_point_).dot(wall_normal_enu_);
            double v_pen = vel_meas.dot(wall_normal_enu_);
            
            // Position projection (prevent wall penetration)
            if (d_pen < 0.0) {
                hybrid_pos = current_pos_est - d_pen * wall_normal_enu_;
            } else {
                hybrid_pos = current_pos_est;
            }
            
            // Velocity projection (kill velocity going into the wall)
            if (v_pen < 0.0) {
                hybrid_vel = vel_meas - v_pen * wall_normal_enu_;
            } else {
                hybrid_vel = vel_meas;
            }
            
        } else {
            if (was_in_contact_) {
                // Transition: Contact -> Flight
                // Recalculate offset to smoothly decouple from the wall
                pos_offset_ = last_hybrid_pos_ - pos_meas;
                was_in_contact_ = false;
                
                RCLCPP_INFO(this->get_logger(), "CONTACT ENDED. Unilateral Constraint Deactivated. New offset: (%.2f, %.2f, %.2f)", 
                            pos_offset_.x(), pos_offset_.y(), pos_offset_.z());
            }
            
            // Free flight: bypass odometry with continuous offset
            hybrid_pos = pos_meas + pos_offset_;
            hybrid_vel = vel_meas; 
            
            // Update biases during free flight (low-pass filter)
            force_bias_ = 0.99 * force_bias_ + 0.01 * last_force_body_;
            filtered_thrust_ += 10.0 * (current_thrust_ - filtered_thrust_) * dt;
            
            Eigen::Quaterniond q_curr_free(quat_meas(0), quat_meas(1), quat_meas(2), quat_meas(3));
            q_curr_free.normalize();
            
            Eigen::Vector3d a_body_raw = (last_force_body_ + filtered_thrust_) / mass_;
            Eigen::Vector3d expected_a_body = q_curr_free.inverse() * Eigen::Vector3d(0.0, 0.0, 9.81);
            Eigen::Vector3d error_a_body = a_body_raw - expected_a_body;
            a_body_bias_ = 0.99 * a_body_bias_ + 0.01 * error_a_body;
        }

        // PUBLISH TO PX4
        
        last_hybrid_pos_ = hybrid_pos;
        publish_to_px4(current_time, hybrid_pos, hybrid_vel, q_bypass);
    }

    void publish_to_px4(const rclcpp::Time& stamp, const Eigen::Vector3d& pos_enu, const Eigen::Vector3d& vel_enu, const Eigen::Quaterniond& q_body_to_enu) {
        px4_msgs::msg::VehicleOdometry out;

        out.timestamp = this->get_clock()->now().nanoseconds() / 1000ULL;
        out.timestamp_sample = stamp.nanoseconds() / 1000ULL;

        Eigen::Vector3d pos_ned = q_enu_to_ned_ * pos_enu;
        out.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        out.position[0] = pos_ned.x();
        out.position[1] = pos_ned.y();
        out.position[2] = pos_ned.z();

        Eigen::Vector3d vel_ned = q_enu_to_ned_ * vel_enu;
        
        out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
        out.velocity[0] = vel_ned.x();
        out.velocity[1] = vel_ned.y();
        out.velocity[2] = vel_ned.z();

        Eigen::Quaterniond q_px4_ned = q_enu_to_ned_ * q_body_to_enu * q_flu_to_frd_;
        out.q[0] = q_px4_ned.w();
        out.q[1] = q_px4_ned.x();
        out.q[2] = q_px4_ned.y();
        out.q[3] = q_px4_ned.z();

        out.position_variance[0] = NAN;
        out.position_variance[1] = NAN;
        out.position_variance[2] = NAN;
        out.orientation_variance[0] = NAN;
        out.orientation_variance[1] = NAN;
        out.orientation_variance[2] = NAN;
        out.velocity_variance[0] = NAN;
        out.velocity_variance[1] = NAN;
        out.velocity_variance[2] = NAN;

        px4_odom_pub_->publish(out);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TactileOdometry>());
    rclcpp::shutdown();
    return 0;
}

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

// #include <mavros_msgs/msg/position_target.hpp>
#include <px4_msgs/msg/tilting_attitude_setpoint.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "boost/thread.hpp"
#include <Eigen/Eigen>
#include "Eigen/Dense"
class MAVROSRep : public rclcpp::Node
{
public:
  MAVROSRep();

  void run();
  void publisher(Eigen::Vector3d p, Eigen::Vector3d v,Eigen::Vector4d q, double yaw, double yawspeed);
  void timerCallback();
  rclcpp::TimerBase::SharedPtr timer_;

private:
    // rclcpp::Subscription< mavros_msgs::msg::PositionTarget>::SharedPtr _mavros_sub;
    rclcpp::Subscription< geometry_msgs::msg::QuaternionStamped>::SharedPtr _quat_sub;
    rclcpp::Subscription< geometry_msgs::msg::PointStamped>::SharedPtr _p_sub;
    rclcpp::Subscription< geometry_msgs::msg::PointStamped>::SharedPtr _v_sub;
    rclcpp::Subscription< geometry_msgs::msg::PointStamped>::SharedPtr _y_sub;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _traj_publisher; 
    rclcpp::Publisher<px4_msgs::msg::TiltingAttitudeSetpoint>::SharedPtr _tilt_publisher;
	Eigen::Vector3d _p, _v;
    Eigen::Vector4d _q;
    double _yaw, _yaw_speed;
    double _rate;
};

MAVROSRep::MAVROSRep(): Node("traj_repub")  {
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&MAVROSRep::timerCallback, this));

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    // _mavros_sub = this->create_subscription< mavros_msgs::msg::PositionTarget >(
    //     "/mavros/setpoint_raw/local_0", qos, [this](const  mavros_msgs::msg::PositionTarget::UniquePtr msg) {
        
    //         _p(0) = msg->position.x;
    //         _p(1) = msg->position.y;
    //         _p(2) = msg->position.z;
    //         _v(0) = msg->velocity.x;
    //         _v(1) = msg->velocity.y;
    //         _v(2) = msg->velocity.z;
    //         _yaw = msg->yaw;
    //         _yaw_speed = msg->yaw_rate;
    // });
    _quat_sub = this->create_subscription< geometry_msgs::msg::QuaternionStamped >(
        "/mpc/des_quat", qos, [this](const  geometry_msgs::msg::QuaternionStamped ::UniquePtr msg) {
        
            _q(0) = msg->quaternion.w;
            _q(1) = msg->quaternion.x;
            _q(2) = msg->quaternion.y;
            _q(3) = msg->quaternion.z;
    });
    _quat_sub = this->create_subscription< geometry_msgs::msg::QuaternionStamped >(
        "/mpc/des_quat", qos, [this](const  geometry_msgs::msg::QuaternionStamped ::UniquePtr msg) {
        
            _q(0) = msg->quaternion.w;
            _q(1) = msg->quaternion.x;
            _q(2) = msg->quaternion.y;
            _q(3) = msg->quaternion.z;
    });
    _p_sub = this->create_subscription< geometry_msgs::msg::PointStamped >(
        "/mpc/des_p", qos, [this](const  geometry_msgs::msg::PointStamped ::UniquePtr msg) {
        
            _p(0) = msg->point.x;
            _p(1) = msg->point.y;
            _p(2) = msg->point.z;
    });
    _v_sub = this->create_subscription< geometry_msgs::msg::PointStamped >(
        "/mpc/des_v", qos, [this](const  geometry_msgs::msg::PointStamped ::UniquePtr msg) {
        
            _v(0) = msg->point.x;
            _v(1) = msg->point.y;
            _v(2) = msg->point.z;
    });
    _y_sub = this->create_subscription< geometry_msgs::msg::PointStamped >(
        "/mpc/des_y", qos, [this](const  geometry_msgs::msg::PointStamped ::UniquePtr msg) {
        
            _yaw = msg->point.x;
            _yaw_speed = msg->point.y;
    });
    
    _rate = 100;
    _traj_publisher = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    _tilt_publisher = this->create_publisher<px4_msgs::msg::TiltingAttitudeSetpoint>("fmu/in/tilting_attitude_setpoint", 10);

}

void MAVROSRep::timerCallback() {
    
    publisher(_p, _v,_q,_yaw,_yaw_speed);
}

void MAVROSRep::publisher(Eigen::Vector3d _p, Eigen::Vector3d _v, Eigen::Vector4d _q, double _yaw, double _yaw_speed) {
	px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	msg.position[0] = _p(0);
	msg.position[1] = _p(1);
	msg.position[2] = _p(2);

	msg.velocity[0] = _v(0);
	msg.velocity[1] = _v(1);
	msg.velocity[2] = _v(2);

   	msg.yaw = _yaw;
	msg.yawspeed = _yaw_speed;

	px4_msgs::msg::TiltingAttitudeSetpoint att_sp{};
	att_sp.timestamp = msg.timestamp;

	att_sp.q_d[0] = _q(0);
	att_sp.q_d[1] = _q(1);
	att_sp.q_d[2] = _q(2);
	att_sp.q_d[3] = _q(3);
	
	_tilt_publisher->publish(att_sp);

	_traj_publisher->publish(msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MAVROSRep>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

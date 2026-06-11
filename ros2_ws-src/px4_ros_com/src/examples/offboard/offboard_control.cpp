#include "offboard_control.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <limits> 

OffboardControl::OffboardControl() : Node("offboard_control"), _state(STOPPED){

    _offboard_control_mode_publisher = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    _trajectory_setpoint_publisher = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    _vehicle_command_publisher = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    _tilting_attitude_setpoint_publisher = this->create_publisher<TiltingAttitudeSetpoint>("/fmu/in/tilting_attitude_setpoint", 10);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    
    _odom_sub =
        this->create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/out/vehicle_odometry", qos,
            [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
                _first_odom = true;
                _attitude = matrix::Quaternionf(msg->q.data()[0], msg->q.data()[1], msg->q.data()[2], msg->q.data()[3]);
                _position = matrix::Vector3f(msg->position[0], msg->position[1], msg->position[2]);
                if(isnanf(_position(0)) || 
                        isnanf(_position(1)) ||
                        isnanf(_position(2))) {
                    RCLCPP_WARN(rclcpp::get_logger("OFFBOARD"), "INVALID POSITION: %10.5f, %10.5f, %10.5f",
                        _position(0), _position(1), _position(2));
                }
            });

    auto timer_callback = [this]() -> void {
        if(!_first_odom)
                return;

        if(!_first_traj){
            firstTraj();
            _first_traj = true;
        }
        
        _trajectory.getNext(_x, _xd, _xdd);
        
        publish_offboard_control_mode();
        publish_trajectory_setpoint();
    };

    timer_ = this->create_wall_timer(10ms, timer_callback);

    sub_fsm_cmd_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/fsm/trajectory_command", 10,
        std::bind(&OffboardControl::fsm_cmd_callback, this, std::placeholders::_1));

    sub_spray_cmd_ = this->create_subscription<std_msgs::msg::Bool>(
        "/command/spawn_aruco", 10,
        std::bind(&OffboardControl::spray_cmd_callback, this, std::placeholders::_1));
}

void OffboardControl::fsm_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 7) {
        RCLCPP_ERROR(this->get_logger(), "FSM Error: Command array too short!");
        return;
    }

    static bool is_armed = false;
    if (!is_armed) {
        RCLCPP_INFO(this->get_logger(), "First command received! Switching to OFFBOARD and ARMING.");
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        this->arm();
        is_armed = true;
    }

    matrix::Vector3f sp;
    sp(0) = msg->data[0]; // X
    sp(1) = msg->data[1]; // Y
    sp(2) = msg->data[2]; // Z
    float roll = static_cast<float>(msg->data[3]);
    float pitch = static_cast<float>(msg->data[4]);
    float yaw = static_cast<float>(msg->data[5]);
    double duration = msg->data[6];

    RCLCPP_INFO(this->get_logger(), "[FSM] Received route to (%.2f, %.2f, %.2f) Yaw: %.2f", sp(0), sp(1), sp(2), yaw);

    if (duration < 0.0) {
        RCLCPP_INFO(this->get_logger(), "[FSM] EMERGENCY STOP! Holding exact current position.");
        startTraj(_position, roll, pitch, yaw, std::abs(duration));
    } else {
        startTraj(sp, roll, pitch, yaw, duration);
    }
}

void OffboardControl::spray_cmd_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data == true) {
        RCLCPP_INFO(this->get_logger(), "[FSM] Activating paint servo!");
        this->publish_vehicle_command(183, 1.0, 2000.0, 0.0);
    } else {
        RCLCPP_INFO(this->get_logger(), "[FSM] Releasing paint servo.");
        this->publish_vehicle_command(183, 1.0, 1000.0, 0.0);
    }
}

void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void OffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void OffboardControl::publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = true;
    msg.acceleration = true;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    _offboard_control_mode_publisher->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint() {
    TrajectorySetpoint msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // Posizione
    msg.position[0] = _x.pose.position.x;
    msg.position[1] = _x.pose.position.y;
    msg.position[2] = _x.pose.position.z;

    // Velocità viva (feedforward vitale per la stabilità)
    msg.velocity[0] = _xd.twist.linear.x;
    msg.velocity[1] = _xd.twist.linear.y;
    msg.velocity[2] = _xd.twist.linear.z;

    // Accelerazione viva
    msg.acceleration[0] = _xdd.accel.linear.x;
    msg.acceleration[1] = _xdd.accel.linear.y;
    msg.acceleration[2] = _xdd.accel.linear.z;

    msg.jerk[0] = std::numeric_limits<float>::quiet_NaN();
    msg.jerk[1] = std::numeric_limits<float>::quiet_NaN();
    msg.jerk[2] = std::numeric_limits<float>::quiet_NaN();

    matrix::Quaternionf des_att(_x.pose.orientation.w, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z);
    msg.yaw = matrix::Eulerf(des_att).psi();
    
    msg.yawspeed = std::numeric_limits<float>::quiet_NaN();

    TiltingAttitudeSetpoint att_sp{};
    att_sp.timestamp = msg.timestamp;
    att_sp.q_d[0] = des_att(0);
    att_sp.q_d[1] = des_att(1);
    att_sp.q_d[2] = des_att(2);
    att_sp.q_d[3] = des_att(3);
    _tilting_attitude_setpoint_publisher->publish(att_sp);

    _trajectory_setpoint_publisher->publish(msg);
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2, float param3) {
    VehicleCommand msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    _vehicle_command_publisher->publish(msg);
}

void OffboardControl::firstTraj() {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    std::vector<double> times;
    geometry_msgs::msg::PoseStamped p;
    double t;

    _last_pos_sp = _position;
    _last_att_sp = _attitude;

    p.pose.position.x = _last_pos_sp(0);
    p.pose.position.y = _last_pos_sp(1);
    p.pose.position.z = _last_pos_sp(2); 

    p.pose.orientation.w = _last_att_sp(0);
    p.pose.orientation.x = _last_att_sp(1);
    p.pose.orientation.y = _last_att_sp(2);
    p.pose.orientation.z = _last_att_sp(3);
    t = 0.0f;

    poses.push_back(p);
    times.push_back(t);
    
    poses.push_back(p);
    times.push_back(0.1);

    _trajectory.set_waypoints(poses, times);
    _trajectory.compute();
}

void OffboardControl::takeoffTraj(float alt) {

    while(_trajectory.isReady()) {
        usleep(0.1e6);
    }

    std::vector<geometry_msgs::msg::PoseStamped> poses;
    std::vector<double> times;
    geometry_msgs::msg::PoseStamped p;
    double t;

    p.pose.position.x = _last_pos_sp(0);
    p.pose.position.y = _last_pos_sp(1);
    p.pose.position.z = _last_pos_sp(2); 

    p.pose.orientation.w = _last_att_sp(0);
    p.pose.orientation.x = _last_att_sp(1);
    p.pose.orientation.y = _last_att_sp(2);
    p.pose.orientation.z = _last_att_sp(3);
    t = 0.0f;

    poses.push_back(p);
    times.push_back(t);
    
    alt = alt > 0 ? -alt : alt;
    p.pose.position.z = alt; 
    poses.push_back(p);
    times.push_back(3.0f * abs(alt));

    _last_pos_sp(2) = alt;

    _trajectory.set_waypoints(poses, times);
    _trajectory.compute();
}

void OffboardControl::startTraj(matrix::Vector3f pos, float roll, float pitch, float yaw, double d) {

    std::vector<geometry_msgs::msg::PoseStamped> poses;
    std::vector<double> times;
    geometry_msgs::msg::PoseStamped p;
    double t;

    roll = abs(roll) > 0.3f ? 0.3f * matrix::sign(roll) : roll;
    pitch = abs(pitch) > 0.3f ? 0.3f * matrix::sign(pitch) : pitch;
    if(pos(2) > 0.0f)
        pos(2) *= -1;

    matrix::Quaternionf att(matrix::Eulerf(roll, pitch, yaw));

    // Si parte dalla VERA posizione fisica attuale per azzerare i gradini/scatti.
    p.pose.position.x = _position(0);
    p.pose.position.y = _position(1);
    p.pose.position.z = _position(2); 

    p.pose.orientation.w = _attitude(0);
    p.pose.orientation.x = _attitude(1);
    p.pose.orientation.y = _attitude(2);
    p.pose.orientation.z = _attitude(3);

    t = 0.0;
    
    poses.push_back(p);
    times.push_back(t);

    p.pose.position.x = pos(0);
    p.pose.position.y = pos(1);
    p.pose.position.z = pos(2); 

    p.pose.orientation.w = att(0);
    p.pose.orientation.x = att(1);
    p.pose.orientation.y = att(2);
    p.pose.orientation.z = att(3);

    t = d;

    _last_pos_sp = pos;
    _last_att_sp = att;

    poses.push_back(p);
    times.push_back(t);

    _trajectory.set_waypoints(poses, times);
    _trajectory.compute();
}

int main(int argc, char* argv[]) {
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto offboardCtrlPtr = std::make_shared<OffboardControl>();
    rclcpp::spin(offboardCtrlPtr);
    rclcpp::shutdown();
    return 0;
}
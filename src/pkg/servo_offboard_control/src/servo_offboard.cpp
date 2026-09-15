#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardPeripheralServo : public rclcpp::Node
{
public:
    OffboardPeripheralServo() : Node("offboard_peripheral_servo")
    {
        n_sec_ = declare_parameter<double>("n_sec", 2.0);

        offboard_pub_ = create_publisher<OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

        traj_pub_ = create_publisher<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        cmd_pub_ = create_publisher<VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        servo_pub_ = create_publisher<ActuatorServos>(
            "/fmu/in/actuator_servos", 10);

        offboard_thread_ = std::thread(&OffboardPeripheralServo::offboard_loop, this);

        RCLCPP_INFO(get_logger(), "READY: digita y per +1, qualsiasi altro carattere chiude il nodo. n_sec=%.2f", n_sec_);
    }

    ~OffboardPeripheralServo() override
    {
        running_ = false;

        if (offboard_thread_.joinable()) {
            offboard_thread_.join();
        }
    }

private:
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
    rclcpp::Publisher<ActuatorServos>::SharedPtr servo_pub_;

    std::thread offboard_thread_;
    std::atomic<bool> running_{true};

    int counter_ = 0;
    double n_sec_{2.0};

    // ---------------- OFFBOARD THREAD ----------------
    void offboard_loop()
    {
        while (running_ && rclcpp::ok()) {
            publish_offboard();
            publish_setpoint();

            if (counter_ == 10) {
                arm();
                set_offboard();
            }

            counter_++;
            std::this_thread::sleep_for(100ms);
        }
    }

    // ---------------- OFFBOARD ----------------
    void publish_offboard()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.attitude = true;
        msg.timestamp = now();
        offboard_pub_->publish(msg);
    }

    void publish_setpoint()
    {
        TrajectorySetpoint sp{};
        sp.position = {0.0f, 0.0f, -5.0f};
        sp.yaw = 0.0f;
        sp.timestamp = now();
        traj_pub_->publish(sp);
    }

    void set_offboard()
    {
        send_cmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        RCLCPP_INFO(get_logger(), "OFFBOARD enabled");
    }

    void arm()
    {
        send_cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO(get_logger(), "ARMED");
    }

	    // ---------------- PERIPHERAL CMD (187) ----------------
	void send_peripheral(float v1)
	{
	    VehicleCommand cmd{};

	    cmd.command = VehicleCommand::VEHICLE_CMD_DO_SET_ACTUATOR;

	    cmd.param1 = v1;    // Forza il valore (-1.0 a 1.0) sul Canale 1 del Set selezionato
	    cmd.param2 = 0.f;
	    cmd.param3 = 0.f;
	    cmd.param4 = 0.f;
	    cmd.param5 = 0.0;
	    cmd.param6 = 0.0;
	    cmd.param7 = 0.0f;  // <-- CORREZIONE: 0.0f indica l'Actuator Set 1 mappato su QGC

	    cmd.target_system = 1;
	    cmd.target_component = 1;
	    cmd.from_external = true;
	    cmd.timestamp = now();

	    cmd_pub_->publish(cmd);

	    RCLCPP_INFO(get_logger(), "PERIPHERAL CMD (187) sent to Set 1: %.2f", v1);
	}

    // ---------------- SERVO AUX1 ----------------
    void send_servo(float value)
    {
        ActuatorServos msg{};

        msg.timestamp = now();
        msg.timestamp_sample = now();

        for (int i = 0; i < 8; i++) {
            msg.control[i] = NAN;
        }

        msg.control[0] = value; // AUX1

        servo_pub_->publish(msg);

        RCLCPP_INFO(get_logger(), "SERVO AUX1: %.2f", value);
    }

    // ---------------- KEYS ----------------
    void handle_keys()
    {
        char c = 0;

        if (std::scanf(" %c", &c) != 1) {
            RCLCPP_ERROR(get_logger(), "Errore nella lettura del comando, chiudo il nodo");
            running_ = false;
            rclcpp::shutdown();
            return;
        }

        if (c == 'y') {
            send_peripheral(-1.0f);
            send_servo(-1.0f);

            RCLCPP_INFO(get_logger(), "Comando y ricevuto: +1 inviato, attendo %.2f s", n_sec_);
            std::this_thread::sleep_for(std::chrono::duration<double>(n_sec_));

            if (!running_ || !rclcpp::ok()) {
                return;
            }

            send_peripheral(1.0f);
            send_servo(1.0f);
            RCLCPP_INFO(get_logger(), "Ripubblicato -1, pronto per il prossimo comando");
            return;
        }

        RCLCPP_INFO(get_logger(), "Comando '%c' non valido, chiudo il nodo", c);
        running_ = false;
        rclcpp::shutdown();
    }

    void run_command_loop()
    {
        while (running_ && rclcpp::ok()) {
            RCLCPP_INFO(get_logger(), "Attendo comando: y per attivare il servo");
            handle_keys();
        }
    }

    // ---------------- SEND CMD ----------------
    void send_cmd(uint16_t cmd, float p1, float p2 = 0.f)
    {
        VehicleCommand msg{};
        msg.command = cmd;
        msg.param1 = p1;
        msg.param2 = p2;

        msg.target_system = 1;
        msg.target_component = 1;
        msg.from_external = true;
        msg.timestamp = now();

        cmd_pub_->publish(msg);
    }

    uint64_t now()
    {
        return this->get_clock()->now().nanoseconds() / 1000;
    }

public:
    void spin_input_loop()
    {
        run_command_loop();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardPeripheralServo>();
    node->spin_input_loop();
    rclcpp::shutdown();
    return 0;
}

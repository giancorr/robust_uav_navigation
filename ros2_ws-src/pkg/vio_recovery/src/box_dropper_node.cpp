#include <chrono>
#include <memory>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>

using namespace std::chrono_literals;

class BoxDropperNode : public rclcpp::Node
{
public:
    BoxDropperNode() : Node("box_dropper_node")
    {
        // Publisher per il comando attuatore
        cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // Sottoscrizione al topic /command/drop_marker emesso dallo stato STOP della FSM
        drop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/command/drop_marker", 10,
            std::bind(&BoxDropperNode::drop_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "BoxDropperNode initialized. Waiting for /command/drop_marker");
    }

private:
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drop_sub_;
    
    // Per evitare aperture multiple accidentali
    std::atomic<bool> is_dropping_{false};

    void drop_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !is_dropping_.exchange(true)) {
            // Avviamo un thread asincrono per l'apertura in modo da non bloccare il callback
            std::thread([this]() {
                RCLCPP_INFO(this->get_logger(), "Box opened.");
                send_peripheral(-1.0f);
                
                // Attendi 2.0 secondi come da parametro hardcoded concordato
                std::this_thread::sleep_for(2000ms);
                
                RCLCPP_INFO(this->get_logger(), "Box closed.");
                send_peripheral(1.0f);
                
                // Resettiamo il flag così può essere triggerato di nuovo in futuro
                is_dropping_ = false;
            }).detach();
        }
    }

    void send_peripheral(float v1)
    {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_ACTUATOR;
        cmd.param1 = v1;
        cmd.param2 = 0.f;
        cmd.param3 = 0.f;
        cmd.param4 = 0.f;
        cmd.param5 = 0.0;
        cmd.param6 = 0.0;
        cmd.param7 = 0.0f;  // Actuator Set 1
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.from_external = true;
        cmd.timestamp = now();
        cmd_pub_->publish(cmd);
    }

    uint64_t now()
    {
        return this->get_clock()->now().nanoseconds() / 1000;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoxDropperNode>());
    rclcpp::shutdown();
    return 0;
}

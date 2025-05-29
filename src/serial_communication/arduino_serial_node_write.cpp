#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <string>

using namespace drivers::serial_driver;

class ArduinoSerialNodeWrite : public rclcpp::Node {
public:
    ArduinoSerialNodeWrite() : Node("arduino_serial_node_write") {
        // Initialize serial port
        try {
            // Create IO context
            io_context_ = std::make_shared<IoContext>();
            
            // Configure serial port parameters
            const SerialPortConfig config {
                115200,        // baudrate
                FlowControl::NONE,
                Parity::NONE,
                StopBits::ONE
            };

            // Create and open serial port
            serial_port_ = std::make_unique<SerialPort>(
                *io_context_,
                "/dev/ttyACM0",  // Device path
                config
            );
            
            serial_port_->open();

            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");

        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Serial port error: %s", e.what());
            rclcpp::shutdown();
        }

        send_to_arduino = this->create_subscription<std_msgs::msg::String>(
            "send_to_arduino", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                static std::string last_text;
                if (msg->data != last_text) {
                    last_text = msg->data;
                    writeSerial(msg->data);
                }
            }
        );
    }

private:
    void writeSerial(const std::string command) {
        try {
            if (serial_port_ && serial_port_->is_open()) {
                // Convert string to byte vector (command already contains newline)
                std::vector<uint8_t> data(command.begin(), command.end());
                
                serial_port_->send(data);
            }
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Write error: %s", e.what());
        }
    }

    // Member variables
    std::shared_ptr<IoContext> io_context_;
    std::unique_ptr<SerialPort> serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr serial_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr send_to_arduino;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoSerialNodeWrite>());
    rclcpp::shutdown();
    return 0;
}
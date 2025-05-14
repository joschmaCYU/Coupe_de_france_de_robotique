#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "geometry_msgs/msg/pose.hpp" 
// #include "coupe_de_france_de_robotique/src/msg/Encoder.hpp" //src/coupe_de_france_de_robotique/src/msg/Encoder.msg
#include <vector>
#include <string>

using namespace drivers::serial_driver;

class ArduinoSerialNodeRead : public rclcpp::Node {
public:
    ArduinoSerialNodeRead() : Node("arduino_serial_node_read"), led_state_(false) {
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
                "/dev/ttyUSB0",  // Device path
                config
            );
            
            serial_port_->open();

            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");

        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Serial port error: %s", e.what());
            rclcpp::shutdown();
        }

        publisher();

        // Initialize publishers and subscribers
        /**temp_publisher_ = this->create_publisher<std_msgs::msg::Float32>("temperature", 10);
        button_publisher_ = this->create_publisher<std_msgs::msg::Bool>("button_state", 10);
        led_publisher_ = this->create_publisher<std_msgs::msg::String>("led_state", 10);
        command_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "serial_command", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) { writeSerial(msg); }
        );

        */

        // Create 1-second timer for LED toggling
        //const std::string command = ;

        // encoder_publisher_ = this->create_publisher<coupe_de_france_de_robotique::msg::Encoder>("encoder", 10);

        serial_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { readSerial(); }
        );

    }

private:
    void processMessage(const std::string &message) {
        std::stringstream ss(message);
        std::string key, value;

        if (std::getline(ss, key, ':') && std::getline(ss, value)) {
            if (key == "ENCODER_VALUES") {
                // float[2] enc_val = {0.0, 0.0};
                // int i = 0;

                // while (std::getline(values, enc_values, ' ')) {
                //     if (i >= 2 ) {
                //         RCLCPP_ERROR(this->get_logger(), "Recived too much encoders value, breaking");
                //         break;
                //     }

                //     enc_val[i] = std::flof(value);
                //     i++;
                // }

                // auto msg = coupe_de_france_de_robotique::msg::Encoder();
                // msg.encoder0 = enc_val[0];
                // msg.encoder1 = enc_val[1];
                // encoder_publisher_->publish(msg);
            } else if (key == "BUTTON") {
                /*auto msg = std_msgs::msg::Bool();
                msg.data = (value == "1");
                button_publisher_->publish(msg);*/
            } else if (key == "TEAMBLUE") {
                auto bool_msg = std_msgs::msg::Bool();
                bool_msg.data = (value != "0");
                teamBlue_pub_->publish(bool_msg);
            } else if (key == "POSE") {
                auto pose_msg = geometry_msgs::msg::Pose();
                pose_msg = value;
                pose_pub_->publish(pose_msg);
            } else if (key == "GRAB") {
                auto grab_msg = std_msgs::msg::Bool();
                grab_msg.data = (value != "0");
                grab_pub_->publish(grab_msg);
            }
        }
    }
    void readSerial() {
        try {
            if (serial_port_->is_open()) {
                // Create buffer for receiving data
                std::vector<uint8_t> buffer(256);
                
                // Receive data (returns number of bytes read)
                const size_t bytes_read = serial_port_->receive(buffer);
                
                if (bytes_read > 0) {
                    // Convert byte vector to string
                    const std::string message(buffer.begin(), buffer.begin() + bytes_read);
                    RCLCPP_INFO(this->get_logger(), "Received: %s", message.c_str());
                    processMessage(message);
                }
            }
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Read error: %s", e.what());
        }
    }

    void publisher() {
        teamBlue_pub_ = this->create_publisher<std_msgs::msg::Bool>("team_blue", 10);
        pose_pub_ = this->create_publisher<std_msgs::msg::String>("send_to_arduino", 10);
        grab_pub_ = this->create_publisher<std_msgs::msg::Bool>("grabbed", 10);
    }

    // Member variables
    std::shared_ptr<IoContext> io_context_;
    std::unique_ptr<SerialPort> serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr serial_timer_;
    // rclcpp::Publisher<coupe_de_france_de_robotique::msg::Encoder>::SharedPtr encoder_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr teamBlue_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grab_pub_
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;
    bool led_state_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoSerialNodeRead>());
    rclcpp::shutdown();
    return 0;
}
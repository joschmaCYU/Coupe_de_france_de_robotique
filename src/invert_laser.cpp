//Copyright (c) 2018, Tim Kambic
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//* Redistributions of source code must retain the above copyright
//	notice, this list of conditions and the following disclaimer.
//* Redistributions in binary form must reproduce the above copyright
//	notice, this list of conditions and the following disclaimer in the
//	documentation and/or other materials provided with the distribution.
//* Neither the name of the <organization> nor the
//	names of its contributors may be used to endorse or promote products
//	derived from this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
//DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <memory>

class InvertLaserNode : public rclcpp::Node {
public:
  InvertLaserNode()
  : Node("invert_laser")
  {
    // declare and read parameter
    this->declare_parameter<std::string>("out_frame", "");
    this->get_parameter("out_frame", out_frame_);

    // create publisher & subscriber
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan_inv",  rclcpp::QoS(5));
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/ldlidar_node/scan", rclcpp::QoS(5),
      std::bind(&InvertLaserNode::scanCB, this, std::placeholders::_1));
  }

private:
  void scanCB(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    // copy incoming message
    auto msg_out = *msg;
    if (!out_frame_.empty()) {
      msg_out.header.frame_id = out_frame_;
    }
    // reverse ranges & intensities
    std::reverse(msg_out.ranges.begin(), msg_out.ranges.end());
    std::reverse(msg_out.intensities.begin(), msg_out.intensities.end());
    publisher_->publish(msg_out);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  std::string out_frame_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InvertLaserNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
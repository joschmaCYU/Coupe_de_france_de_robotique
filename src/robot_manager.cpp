#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose.hpp"  // Added include
#include <cstdlib>  // for std::system

class RobotManager : public rclcpp::Node
{
public:
  RobotManager() : Node("robot_manager"), grab_started_(false), nav2_sent_(false) {
    // Check if blue or yellow team
    // TODO 
    // make a file of all the can coords
    // find the closest cans
    // if all there take
      // go to depo spot
    // else 
    // find other closest cans

    // Launch the first navigation with a "pose1" argument.
    std::system("ros2 run robot_creation example_nav_to_pose.py");

    // Subscribe to /arrived topic.
    arrived_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/arrived", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !grab_started_) {
          RCLCPP_INFO(this->get_logger(), "Received /arrived true; launching grab.py ...");
          //std::system("ros2 run robot_creation grab.py");
          grab_started_ = true;
        }
      }
    );

    // Subscribe to /grabed topic.
    grabed_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/grabed", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !nav2_sent_) {
          RCLCPP_INFO(this->get_logger(), "Received /grabed true; launching second navigation (pose2) ...");
          std::system("ros2 run robot_creation example_nav_to_pose.py pose2");
          nav2_sent_ = true;
        }
      }
    );

    // Create publisher for "go_to_pose" topic
    go_to_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("go_to_pose", 10);
    // Setup a timer to publish a Pose message
    pose_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RobotManager::publish_pose, this)
    );
  }

private:
  // Method to publish a Pose message to the "go_to_pose" topic.
  void publish_pose() {
    geometry_msgs::msg::Pose pose_msg;
    // ...set pose_msg fields as needed...
    pose_msg.position.x = 1.0;
    pose_msg.position.y = 2.0;
    pose_msg.position.z = 0.0;
    pose_msg.orientation.x = 0.0;
    pose_msg.orientation.y = 0.0;
    pose_msg.orientation.z = 0.0;
    pose_msg.orientation.w = 1.0;
    go_to_pose_pub_->publish(pose_msg);
    // Optionally, cancel timer if one-time publish is desired
    // pose_timer_->cancel();
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrived_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grabed_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr go_to_pose_pub_; // New publisher
  rclcpp::TimerBase::SharedPtr pose_timer_;  // New timer for publishing
  bool nav1_sent_;
  bool grab_started_;
  bool nav2_sent_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

//TODO test with nav2 on
// ros2 run twist_mux twist_mux --ros-args --param-file ./src/robot_creation/src/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped
// ros2 launch robot_creation online_async_launch.py use_sim_time:=true params_file:=./src/robot_creation/src/config/my_controllers.yaml
// 
// launc sim
// ros2 launch robot_creation localization_launch.py map:=./my_map_save.yaml use_sime_time:=true (amcl)
// rviz map /map transient
// ros2 launch robot_creation navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true (nav2) 
// rviz add costmap

// real + slam + nav2(use sim time false) + rviz2 + teleop
// in rviz add map topic costmap color scheme

#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNTP = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

class MultiPoseNavigator : public rclcpp::Node
{
public:
  MultiPoseNavigator()
  : Node("multi_pose_navigator"), first_pose_arrived_(false), last_cylinder_value_(0) {
    // Create the action client for the "navigate_through_poses" action
    action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");

    // Subscribe to the "cylinder" topic (assumed type std_msgs::msg::Int32)
    cylinder_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "cylinder", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        last_cylinder_value_ = msg->data;
      }
    );
  }

  bool wait_for_server()
  {
    // Wait for the action server to be available
    if (!action_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(get_logger(), "NavigateThroughPoses action server not available after waiting");
      return false;
    }
    return true;
  }

  void send_goal(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
  {
    auto goal_msg = NavigateThroughPoses::Goal();
    goal_msg.poses = poses;  // Set the vector of goal poses

    RCLCPP_INFO(get_logger(), "Sending multiple poses goal...");

    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.feedback_callback =
      [this](GoalHandleNTP::SharedPtr,
             const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Feedback: Distance remaining: %.2f", feedback->distance_remaining);
        // Check if robot has arrived at first pose (threshold: 0.25) and perform cylinder check only once
        if (!first_pose_arrived_ && feedback->distance_remaining <= 0.3) {
          first_pose_arrived_ = true;
          if (last_cylinder_value_ == 4) {
            RCLCPP_INFO(this->get_logger(), "Cylinder condition met: value is 4");
          } else {
            RCLCPP_ERROR(this->get_logger(), "Cylinder condition not met: value is %d", last_cylinder_value_);
          }
        }
      };

    // Send goal asynchronously and wait until it is accepted
    auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle)
          != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send goal");
      return;
    }
    auto goal_handle = future_goal_handle.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
      return;
    }

    // Wait for the result
    auto future_result = action_client_->async_get_result(goal_handle);
    RCLCPP_INFO(get_logger(), "Waiting for result...");
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result)
          != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get result");
      return;
    }
    auto result = future_result.get();
    switch(result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Navigation through poses succeeded!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Navigation through poses aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Navigation through poses canceled");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        break;
    }
  }

private:
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr action_client_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cylinder_sub_;  // New subscription
  int last_cylinder_value_;  // Store last value from "cylinder" topic
  bool first_pose_arrived_;  // Flag to check if first pose has been reached
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiPoseNavigator>();

  if (!node->wait_for_server()) {
    rclcpp::shutdown();
    return 1;
  }

  // Create a vector of goal poses
  std::vector<geometry_msgs::msg::PoseStamped> goal_poses;

  // First goal
  geometry_msgs::msg::PoseStamped pose1;
  pose1.header.frame_id = "map";
  pose1.header.stamp = node->now();
  pose1.pose.position.x = 0.5;
  pose1.pose.position.y = 0.0;
  pose1.pose.position.z = 0.05;
  pose1.pose.orientation.x = 0.0;
  pose1.pose.orientation.y = 0.0;
  pose1.pose.orientation.z = 0.0;
  pose1.pose.orientation.w = 1.0;
  goal_poses.push_back(pose1);

  // Second goal
  geometry_msgs::msg::PoseStamped pose2;
  pose2.header.frame_id = "map";
  pose2.header.stamp = node->now();
  pose2.pose.position.x = 0.5;
  pose2.pose.position.y = 0.5;
  pose2.pose.position.z = 0.05;
  pose2.pose.orientation.x = 0.0;
  pose2.pose.orientation.y = 0.0;
  pose2.pose.orientation.z = 0.0;
  pose2.pose.orientation.w = 1.0;
  goal_poses.push_back(pose2);

  // (Add more poses if needed)

  node->send_goal(goal_poses);

  rclcpp::shutdown();
  return 0;
}

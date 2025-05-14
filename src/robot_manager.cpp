#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose.hpp"  // Added include
#include <cstdlib>  // for std::system
#include <cstdio>   // for std::snprintf

bool teamBlue = false;

class RobotManager : public rclcpp::Node {
public:

  float goal_pose_X = 0.0;
  float goal_pose_Y = 0.0;
  float goal_pose_theta = 0.0;
  float initial_pose_X = 0.0;
  float initial_pose_Y = 0.0;
  float initial_pose_theta = 0.0;

  int got_cans = 0;

  bool can_in_hands = false;

  RobotManager() : Node("robot_manager"), grab_started_(false) {
    // Check if blue or yellow team
    // TODO 
    // make a file of all the can coords
    // find the closest cans
    // if all there take
      // go to depo spot
    // else 
    // find other closest cans

    subscribers();

                // TODO remove this from main needs to be called when needed

    // if blue start 0.225, 0.875
    // if yellow start 2.775, 0.875

    if (got_cans == 0 && !grab_started_ && !can_in_hands) {
      if (teamBlue) {
        initial_pose_X = 0.225;
        initial_pose_Y = 0.875;
        initial_pose_theta = 1; // look top
        //0.075 0.4 0 (center of BR)
        goal_pose_X = 0.150; // go a bit above (more ?)
        goal_pose_Y = 0.4;
        goal_pose_theta = 0.0;
      } else {
        initial_pose_X = 2.775;
        initial_pose_Y = 0.875;
        initial_pose_theta = 0; // look bottom
        
        // 2.925, 0.4 0 (center of TR)
        goal_pose_X = 2.850; // go a bit under (more ?)
        goal_pose_Y = 0.4;
        goal_pose_theta = 1;
      }

      char cmd[256];
      std::snprintf(cmd, sizeof(cmd),
        "ros2 run robot_creation example_nav_to_pose.py --intitial_pose %f,%f,%f --pose %f,%f,%f",
        initial_pose_X, initial_pose_Y, initial_pose_theta,
        goal_pose_X, goal_pose_Y, goal_pose_theta
      );
      std::system(cmd);
    } else if (got_cans == 1) {
      // TODO
      // find the closest cans
      // if all there take
      // go to depo spot
    }

    if (can_in_hands) {
      // TODO
      if (got_cans == 0) {
        if (teamBlue) {
          initial_pose_X = goal_pose_X;
          initial_pose_Y = goal_pose_Y;
          initial_pose_theta = goal_pose_theta;
          
          // 0.225, 0.1
          goal_pose_X = 0.225;
          goal_pose_Y = 0.2; // a bit left
          goal_pose_theta = 0.7071068; // look right
        }
      }
    }
  }

  void subscribers() {
    // Subscribe to /grabed topic.
    grabed_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/grabed", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data) {
      RCLCPP_INFO(this->get_logger(), "Received /grabed true; launching second navigation (pose2) ...");
      std::system("ros2 run robot_creation example_nav_to_pose.py pose2");
        grab_started_ = false;
        can_in_hands = true;
      }
    }
    );

    // Subscribe to /arrived topic.
    arrived_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/arrived", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !grab_started_) {
        RCLCPP_INFO(this->get_logger(), "Received /arrived true; launching grab.py ...");
          std::system("ros2 run robot_creation grab.py");
          if (can_in_hands) {
            can_in_hands = false;
            got_cans += 1;
          } else {
            // TODO write serial command to start grabbing
            grab_started_ = true;
          }    
        }
      }
    );

    // Subscribe to /teamBlue topic.
    team_blue_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/teamBlue", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        teamBlue = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received /teamBlue: %s", teamBlue ? "true" : "false");
      }
    );

    // Subscribe to /pose topic.
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/pose", 10,
      [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received /pose: position(x: %f, y: %f, z: %f), orientation(x: %f, y: %f, z: %f, w: %f)",
            msg->position.x, msg->position.y, msg->position.z,
            msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        initial_pose_X = msg->position.x;
        initial_pose_Y = msg->position.y;
        initial_pose_theta = msg->orientation.z;
      }
    );
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrived_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grabed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr team_blue_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;

  bool grab_started_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  for (int i = 0; i < argc; ++i) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "argv[%d]: %s", i, argv[i]);
  }
  auto node = std::make_shared<RobotManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

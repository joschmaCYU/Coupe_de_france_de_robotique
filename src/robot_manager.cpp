#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream> // for std::stringstream

#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <sys/wait.h> // wait
#include <unistd.h>   // fork, execlp

bool teamBlue = false;

class RobotManager : public rclcpp::Node {
public:
  float goal_pose_X = 0.0;
  float goal_pose_Y = 0.0;
  float goal_pose_Z_theta = 0.0;
  float goal_pose_W_theta = 0.0;
  float initial_pose_X = 0.0;
  float initial_pose_Y = 0.0;
  float initial_pose_Z_theta = 0.0;
  float initial_pose_W_theta = 0.0;

  int got_cans = 0;

  bool can_in_hands = false;

  RobotManager() : Node("robot_manager") {
    // Check if blue or yellow team
    // TODO
    // make a file of all the can coords (center) && depo spot
    // find the closest cans
    // if all there take
    // go to depo spot
    // else
    // find other closest cans
    publisher();
    subscribers();

    // TODO remove this from main needs to be called when needed

    // if blue start 0.225, 0.875
    // if yellow start 2.775, 0.875
    if (teamBlue) {
      // match just started go to grab cans
      if (got_cans == 0 && !can_in_hands) {
        auto init = readPose("Blue.I");
        initial_pose_X = init[0];
        initial_pose_Y = init[1];
        initial_pose_Z_theta = init[2];
        initial_pose_W_theta = init[3];
        auto goal = readPose("Blue.C1");
        goal_pose_X = goal[0];
        goal_pose_Y = goal[1];
        goal_pose_Z_theta = goal[2];
        goal_pose_W_theta = goal[3];
      }

      if (got_cans == 0 && can_in_hands) {
        auto init = readPose("Blue.C1");
        initial_pose_X = init[0];
        initial_pose_Y = init[1];
        initial_pose_Z_theta = init[2];
        initial_pose_W_theta = init[3];
        auto goal = readPose("Blue.D1");
        goal_pose_X = goal[0];
        goal_pose_Y = goal[1];
        goal_pose_Z_theta = goal[2];
        goal_pose_W_theta = goal[3];
      }
    } else {
      static const std::vector<std::pair<std::string, std::string>> seq = {
          {"I", "C1"},  {"C1", "D1"}, {"D1", "C2"},
          {"C2", "D2"}, {"D2", "C3"}, {"C3", "D3"}};

      // Compute index: two states per can count (before grab, after grab)

      size_t idx = got_cans * 2 + (can_in_hands ? 1 : 0);
      if (idx < seq.size()) {
        auto init_label = std::string("Yellow.") + seq[idx].first;
        auto goal_label = std::string("Yellow.") + seq[idx].second;

        auto init = readPose(init_label);
        initial_pose_X = init[0];
        initial_pose_Y = init[1];
        initial_pose_Z_theta = init[2];
        initial_pose_W_theta = init[3];

        auto goal = readPose(goal_label);
        goal_pose_X = goal[0];
        goal_pose_Y = goal[1];
        goal_pose_Z_theta = goal[2];
        goal_pose_W_theta = goal[3];
      }

      RCLCPP_INFO(this->get_logger(), "got_cans: %d", got_cans);
      RCLCPP_INFO(this->get_logger(), "initial_pose: %f, %f, %f, %f",
                  initial_pose_X, initial_pose_Y, initial_pose_Z_theta,
                  initial_pose_W_theta);
      RCLCPP_INFO(this->get_logger(), "goal_pose: %f, %f, %f, %f", goal_pose_X,
                  goal_pose_Y, goal_pose_Z_theta, goal_pose_W_theta);

      // sendPose();
    }
  }

  void sendPose() {
    pid_t pid = fork();

    if (pid == 0) {
      char init_pose_X_str[32];
      std::snprintf(init_pose_X_str, 32, "%f", initial_pose_X);
      char init_pose_Y_str[32];
      std::snprintf(init_pose_Y_str, 32, "%f", initial_pose_Y);
      char init_pose_Z_theta_str[32];
      std::snprintf(init_pose_Z_theta_str, 32, "%f", initial_pose_Z_theta);
      char init_pose_W_theta_str[32];
      std::snprintf(init_pose_W_theta_str, 32, "%f", initial_pose_W_theta);

      char goal_pose_X_str[32];
      std::snprintf(goal_pose_X_str, 32, "%f", goal_pose_X);
      char goal_pose_Y_str[32];
      std::snprintf(goal_pose_Y_str, 32, "%f", goal_pose_Y);
      char goal_pose_Z_theta_str[32];
      std::snprintf(goal_pose_Z_theta_str, 32, "%f", goal_pose_Z_theta);
      char goal_pose_W_theta_str[32];
      std::snprintf(goal_pose_W_theta_str, 32, "%f", goal_pose_W_theta);

      char pose_msg[256];
      std::snprintf(pose_msg, sizeof(pose_msg), "%s,%s,%s,%s", init_pose_X_str,
                    init_pose_Y_str, init_pose_Z_theta_str,
                    init_pose_W_theta_str);
      char goal_msg[256];
      std::snprintf(goal_msg, sizeof(goal_msg), "%s,%s,%s,%s", goal_pose_X_str,
                    goal_pose_Y_str, goal_pose_Z_theta_str,
                    goal_pose_W_theta_str);
      execlp("ros2", "ros2", "run", "robot_creation", "example_nav_to_pose.py",
             "--initial_pose", pose_msg, "--pose", goal_msg, (char *)nullptr);
    } else if (pid > 0) {
      wait(nullptr); // wait for the child process
    } else {
      perror("fork failed");
    }
  }

  void subscribers() {
    // Subscribe to /grabed topic.
    grabed_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/grabed", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            RCLCPP_INFO(this->get_logger(),
                        "Received /grabed true; launching second navigation "
                        "(pose2) ...");
            tring_to_grab = false;
            can_in_hands = true;
            // Go to pose to depo cans
          }
        });

    // Subscribe to /arrived topic.
    arrived_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/arrived", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data && !tring_to_grab) {
            RCLCPP_INFO(this->get_logger(),
                        "Received /arrived true; sending grab.py ...");
            if (can_in_hands) {
              can_in_hands = false;
              got_cans += 1;
            } else {
              tring_to_grab = true;
              std_msgs::msg::String grab_msg;
              grab_msg.data = "GRAB";
              send_arduino_pub_->publish(grab_msg);
            }
          }
        });

    // Subscribe to /teamBlue topic.
    team_blue_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/team_blue", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
          teamBlue = msg->data;
          RCLCPP_INFO(this->get_logger(), "Received /team_blue: %s",
                      teamBlue ? "true" : "false");
        });

    // Subscribe to /cmd_vel topic.
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          std_msgs::msg::String out_msg;
          std::stringstream ss;
          // CMDVEL:1.0,0.0
          ss << "CMDVEL:" << msg->linear.x << "," << msg->angular.z << ";";
          out_msg.data = ss.str();
          send_arduino_pub_->publish(out_msg);
        });
  }

  void publisher() {
    send_arduino_pub_ =
        this->create_publisher<std_msgs::msg::String>("send_to_arduino", 10);
  }

private:
  std::array<float, 4> readPose(const std::string &label) {
    std::ifstream file(
        "/home/josch/ROS2/src/robot_creation/src/worlds/position_alone.txt"); // ensure this file lists lines like
                                        // "Yellow.I 0.225,0.875,1.0,0.0"
    std::string line;
    while (std::getline(file, line)) {
      if (line.rfind(label, 0) == 0) {
        // strip label and commas
        std::string coords = line.substr(label.size());
        std::replace(coords.begin(), coords.end(), ',', ' ');
        std::istringstream iss(coords);
        std::array<float, 4> p{0, 0, 0, 0};
        iss >> p[0] >> p[1] >> p[2] >> p[3];
        return p;
      }
    }
    return {0, 0, 0, 0};
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrived_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grabed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr team_blue_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr send_arduino_pub_;

  bool tring_to_grab = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

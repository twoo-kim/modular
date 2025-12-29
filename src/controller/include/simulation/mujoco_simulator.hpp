#pragma once
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cmath>
#include <time.h>

#include "simulation/viewer.hpp"
#include "data/config.hpp"
#include "data/module.hpp"
#include "utils/utils.hpp"
#include "utils/xml_utils.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "modular_msgs/msg/state_msg.hpp"
#include "modular_msgs/msg/control_msg.hpp"

/* Mujoco Simulator Class */
class MujocoSimulator : public rclcpp::Node {
public:
  // Constructor and Destructor
  MujocoSimulator();
  ~MujocoSimulator();

  // Run one step
  void run();

private:
  // Callback function
  void controlCallback(const modular_msgs::msg::ControlMsg::SharedPtr msg);
  // Utility function
  std::vector<Module> getModules();
  void publishPose();

  // Configuration
  SimConfig config_;

  // Mujoco
  mjModel* model_{nullptr};
  mjData* data_{nullptr};
  
  // Viewer
  std::unique_ptr<MujocoViewer> viewer_;

  // List of modules
  std::vector<Module> modules_;

  // ROS2
  rclcpp::Publisher<modular_msgs::msg::StateMsg>::SharedPtr pose_pub_;
  std::vector<rclcpp::Subscription<modular_msgs::msg::ControlMsg>::SharedPtr> control_subs_;
};

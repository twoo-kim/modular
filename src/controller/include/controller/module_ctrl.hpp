#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "data/module.hpp"
#include "utils/utils.hpp"

#include "modular_msgs/msg/state_msg.hpp"
#include "modular_msgs/msg/control_msg.hpp"

class CtrlNode : public rclcpp::Node {
public:
  explicit CtrlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  virtual void run();

protected:
  // Callback functions
  virtual void poseCallback(const modular_msgs::msg::StateMsg::SharedPtr msg);

  /* State */
  Module module_;
  
  /* ROS2 */
  rclcpp::Subscription<modular_msgs::msg::StateMsg>::SharedPtr pose_sub_;
  rclcpp::Publisher<modular_msgs::msg::ControlMsg>::SharedPtr ctrl_pub_;
};

#include "controller/module_ctrl.hpp"
using std::placeholders::_1;

CtrlNode::CtrlNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("ctrl_node", options) {
  // Initialize
  int index;
  this->declare_parameter<int>("index", -1);
  this->get_parameter("index", index);

  std::string pf = "r"+std::to_string(index)+"_";
  module_.prefix = pf;
  module_.module_id = index;
  
  // Subscriber and Publisher
  std::string topic = pf + "module/control";
  pose_sub_ = this->create_subscription<modular_msgs::msg::StateMsg>("/pose_topic", 10, std::bind(&CtrlNode::poseCallback, this, _1));
  ctrl_pub_ = this->create_publisher<modular_msgs::msg::ControlMsg>(topic, 1);
}

void CtrlNode::poseCallback(const modular_msgs::msg::StateMsg::SharedPtr msg) {
  //
}

void CtrlNode::run() {
  static int i = 0;
  
  // Publish control; Currently simple oscillation
  modular_msgs::msg::ControlMsg msg;
  msg.index = module_.module_id;
  msg.ctrl = M_PI/4 * std::sin(M_PI/100*i++);
  msg.phi = 0;

  ctrl_pub_->publish(msg);
}


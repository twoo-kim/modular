#include "controller/module_ctrl.hpp"

/* Main node */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CtrlNode>();
  rclcpp::Rate rate(100); // 100Hz

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->run();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

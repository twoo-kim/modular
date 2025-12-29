#include "simulation/mujoco_simulator.hpp"

/* Main node */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MujocoSimulator>();
  rclcpp::Rate rate(1000); // 1000Hz

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->run();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
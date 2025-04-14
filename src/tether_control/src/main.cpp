#include "tether_control/tether_control.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  const char *node_user = std::getenv("USER");
  auto node = std::make_shared<tether_control::TetherControl>(
    node_user ? std::string(node_user) + "_tether_control_node" : "tether_control_node");
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
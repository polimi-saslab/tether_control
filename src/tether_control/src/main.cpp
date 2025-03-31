#include "tether_control/tether_control.hpp"

int main(int argc, char *argv[])
{
  std::cout << "Starting tether control node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  const char *node_user = std::getenv("USER");
  auto node = std::make_shared<tether_control::TetherControl>(node_user ? std::string(node_user) + "_tether_control"
                                                                        : "tether_control");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
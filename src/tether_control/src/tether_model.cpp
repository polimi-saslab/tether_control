#include "tether_model/tether_model.hpp"

using namespace std::chrono;

namespace tether_model
{
  TetherModel::TetherModel(const std::string &nodeName) : Node(nodeName)
  {
    RCLCPP_INFO(this->get_logger(), "################# TETHER MODEL NODE INITIALIZING #################");

    // Init publishers
    tether_force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/drone/tether_force", 10);

    auto timer_alive_callback = [this]() -> void {
      if(this->is_node_alive)
        {
          RCLCPP_INFO_ONCE(this->get_logger(), "NODE ALIVE");
          return;
        }
      else
        {
          RCLCPP_INFO(this->get_logger(), "Tether not alive ...");
        }
    };
    timer_alive_ = this->create_wall_timer(10ms, timer_alive_callback);

    RCLCPP_INFO(this->get_logger(), "################# TETHER MODEL NODE INITIALIZED #################");
  }
}

int main(int argc, char *argv[])
{
  std::cout << "Starting tether model node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  const char *node_user = std::getenv("USER");
  auto node = std::make_shared<tether_model::TetherModel>(node_user ? std::string(node_user) + "_tether_model"
                                                                    : "tether_model");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
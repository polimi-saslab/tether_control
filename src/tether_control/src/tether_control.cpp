#include "tether_control/tether_control.hpp"

namespace tether_control
{

  TetherControl::TetherControl(const std::string &nodeName) : Node(nodeName)
  {
    // Initialize the publisher
    // serial_write_pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_write", 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TetherControl::timer_callback,
                                                                        this)); // Callback function bound to the node
  }



  void TetherControl::timer_callback()
  {
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data[0]);
  }
} // namespace tether_control

#ifndef TETHER_CONTROL_HPP
#define TETHER_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace tether_control
{

  class TetherControl : public rclcpp::Node
  {
  public:
    explicit TetherControl(const std::string &nodeName);

  private:
    // publishers
    // rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_write_pub;

    // subscribers examples
    // rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_read_sub;

    // timers
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
  };

} // namespace tether_control

#endif // TETHER_CONTROL_HPP
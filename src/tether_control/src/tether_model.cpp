#include "tether_model/tether_model.hpp"

using namespace std::chrono;

namespace tether_model
{
  TetherModel::TetherModel(const std::string &nodeName) : Node(nodeName)
  {
    RCLCPP_INFO(this->get_logger(), "################# TETHER MODEL NODE INITIALIZING #################");

    // Init publishers
    tether_force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/drone/tether_force", 10);

    // PX4 requires specific QoS, see
    // https://docs.px4.io/main/en/ros2/user_guide.html#compatibility-issues
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Init subscribers
    sim_status_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/tether_control/sim_status", qos, std::bind(&TetherModel::simStatusSubCb, this, std::placeholders::_1));

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

    auto timer_callback = [this]() -> void {
      if(!this->is_node_alive)
        {
          return;
        }
      else
        {
          if(sim_status[0] == 0)
            {
              RCLCPP_INFO(this->get_logger(), "Waiting for simulation mode to be set");
            }
          else
            {
              RCLCPP_INFO(this->get_logger(), "ALIVE");
            }
        }
    };

    timer_ = this->create_wall_timer(10ms, timer_callback);
    timer_alive_ = this->create_wall_timer(1000ms, timer_alive_callback);

    RCLCPP_INFO(this->get_logger(), "################# TETHER MODEL NODE INITIALIZED #################");
  }

  void TetherModel::simStatusSubCb(const std_msgs::msg::UInt8MultiArray msg)
  {
    this->sim_status = msg.data;
    RCLCPP_INFO_ONCE(this->get_logger(), "Received Sim Status %d", this->sim_status[0]);
  }

} // namespace tether_model

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
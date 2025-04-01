#ifndef APPLY_FORCE_PLUGIN_HPP_
#define APPLY_FORCE_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/wrench.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <gz/sim/Server.hh>

namespace gz
{
  namespace sim
  {
    class ApplyForcePlugin : public SystemInterface
    {
    public:
      ApplyForcePlugin();
      ~ApplyForcePlugin() override;

      void Configure(const gz::sim::Entity &entity) override;
      void Update(const gz::sim::UpdateInfo &info) override;

    private:
      void forceCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);

      gz::sim::LinkPtr link_;                                                  // Link to apply force to
      rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr force_sub_; // ROS2 subscriber
      rclcpp::Node::SharedPtr node_;                                           // ROS2 node
    };
  } // namespace sim
} // namespace gz

#endif // APPLY_FORCE_PLUGIN_HPP_
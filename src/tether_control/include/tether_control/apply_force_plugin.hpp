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
    class ApplyForcePlugin : public System
    {
    public:
      ApplyForcePlugin();
      virtual ~ApplyForcePlugin();

      // Load the plugin
      void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf) override;

      // ROS 2 Callback to apply force
      void ApplyForceCallback(const geometry_msgs::msg::Wrench::SharedPtr msg);

    private:
      // ROS 2 Node for subscription
      rclcpp::Node::SharedPtr node_;

      // ROS Transport Node
      gz::transport::Node gz_node_;

      // Subscriber to apply forces
      gz::transport::SubscriberPtr force_subscriber_;

      // Entity representing the drone in Gazebo
      gz::sim::Entity entity_;
    };
  } // namespace sim
} // namespace gz

#endif // APPLY_FORCE_PLUGIN_HPP_

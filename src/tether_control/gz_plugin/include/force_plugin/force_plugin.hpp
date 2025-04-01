#ifndef FORCE_PLUGIN_HPP
#define FORCE_PLUGIN_HPP

#include <gz/sim/System.hh>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace force_plugin
{
  class ForcePlugin : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
  {
  public:
    ForcePlugin();
    ~ForcePlugin() override;

    void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;
    void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override;

  private:
    void OnTwistMsg(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
    gz::sim::Entity baseLinkEntity_;
    double forceX_, forceY_, torqueZ_;
    std::thread rclcpp_thread_;
  };
}

#endif // FORCE_PLUGIN_HPP

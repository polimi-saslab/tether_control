#include "force_plugin/force_plugin.hpp"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

GZ_ADD_PLUGIN(force_plugin::ForcePlugin, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)

namespace force_plugin
{
  ForcePlugin::ForcePlugin() : forceX_(0), forceY_(0), torqueZ_(0) {}

  ForcePlugin::~ForcePlugin()
  {
    if(rclcpp_thread_.joinable())
      rclcpp_thread_.join();
  }

  void ForcePlugin::Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                              gz::sim::EntityComponentManager &ecm, gz::sim::EventManager & /*eventMgr*/)
  {
    auto model = gz::sim::Model(entity);

    // Read body name from SDF
    std::string bodyName = "base_link";
    if(sdf->HasElement("bodyName"))
      bodyName = sdf->Get<std::string>("bodyName");

    baseLinkEntity_ = model.LinkByName(ecm, bodyName);
    if(!baseLinkEntity_)
      {
        RCLCPP_ERROR(rclcpp::get_logger("force_plugin"), "Link [%s] not found in model [%s]", bodyName.c_str(),
                     model.Name(ecm).c_str());
        return;
      }

    // Read topic name from SDF
    std::string cmdTopic = "cmd_vel";
    if(sdf->HasElement("cmdTopic"))
      cmdTopic = sdf->Get<std::string>("cmdTopic");

    node_ = std::make_shared<rclcpp::Node>("force_plugin");
    sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      cmdTopic, 10, std::bind(&ForcePlugin::OnTwistMsg, this, std::placeholders::_1));

    // Create a separate thread to spin the ROS node
    rclcpp_thread_ = std::thread([this]() { rclcpp::spin(node_); });
  }

  void ForcePlugin::PreUpdate(const gz::sim::UpdateInfo & /*info*/, gz::sim::EntityComponentManager &ecm)
  {
    auto linVelComp = ecm.Component<gz::sim::components::LinearVelocityCmd>(baseLinkEntity_);
    auto angVelComp = ecm.Component<gz::sim::components::AngularVelocityCmd>(baseLinkEntity_);

    if(linVelComp && angVelComp)
      {
        linVelComp->Data().Set(forceX_, forceY_, 0);
        angVelComp->Data().Set(0, 0, torqueZ_);
      }
    else
      {
        ecm.CreateComponent(baseLinkEntity_,
                            gz::sim::components::LinearVelocityCmd(gz::math::Vector3d(forceX_, forceY_, 0)));
        ecm.CreateComponent(baseLinkEntity_,
                            gz::sim::components::AngularVelocityCmd(gz::math::Vector3d(0, 0, torqueZ_)));
      }
  }

  void ForcePlugin::OnTwistMsg(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    forceX_ = msg->twist.linear.x;
    forceY_ = msg->twist.linear.y;
    torqueZ_ = msg->twist.angular.z;
  }
}

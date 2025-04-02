#include "force_plugin/force_plugin.hpp"
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

GZ_ADD_PLUGIN(force_plugin::ForcePlugin, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)

namespace force_plugin
{
  ForcePlugin::ForcePlugin() : force_(0, 0, 0), torque_(0, 0, 0) {}

  ForcePlugin::~ForcePlugin()
  {
    if(rclcpp_thread_.joinable())
      rclcpp_thread_.join();
  }

  void ForcePlugin::Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                              gz::sim::EntityComponentManager &ecm, gz::sim::EventManager & /*eventMgr*/)
  {
    auto model = gz::sim::Model(entity);

    // if ros2 not initialised then init it
    if(!rclcpp::ok())
      {
        rclcpp::init(0, nullptr);
      }

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

    // // Read topic name from SDF
    std::string tetherForceTopic = "cmd_vel";
    if(sdf->HasElement("tetherForceTopic"))
      tetherForceTopic = sdf->Get<std::string>("tetherForceTopic");

    node_ = std::make_shared<rclcpp::Node>("force_plugin");
    sub_ = node_->create_subscription<geometry_msgs::msg::Wrench>(
      tetherForceTopic, 10, std::bind(&ForcePlugin::OnWrenchMsg, this, std::placeholders::_1));

    // Create a separate thread to spin the ROS node
    rclcpp_thread_ = std::thread([this]() { rclcpp::spin(node_); });
  }

  // frequency is determined by world sdf variable: max_step_size (which is of 0.004 = 250Hz at the moment of implem)
  void ForcePlugin::PreUpdate(const gz::sim::UpdateInfo & /*info*/, gz::sim::EntityComponentManager &ecm)
  {
    if(ecm.EntityHasComponentType(gz::sim::components::ExternalWorldWrenchCmd::typeId, baseLinkEntity_))
      {
        auto wrenchComp = ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(baseLinkEntity_);
        wrenchComp->Data().mutable_force()->set_x(force_.X());
        wrenchComp->Data().mutable_force()->set_y(force_.Y());
        wrenchComp->Data().mutable_force()->set_z(force_.Z());
        wrenchComp->Data().mutable_torque()->set_x(torque_.X());
        wrenchComp->Data().mutable_torque()->set_y(torque_.Y());
        wrenchComp->Data().mutable_torque()->set_z(torque_.Z());
      }
    else
      {
        gz::msgs::Wrench wrenchMsg;
        wrenchMsg.mutable_force()->set_x(force_.X());
        wrenchMsg.mutable_force()->set_y(force_.Y());
        wrenchMsg.mutable_force()->set_z(force_.Z());
        wrenchMsg.mutable_torque()->set_x(torque_.X());
        wrenchMsg.mutable_torque()->set_y(torque_.Y());
        wrenchMsg.mutable_torque()->set_z(torque_.Z());

        ecm.CreateComponent(baseLinkEntity_, gz::sim::components::ExternalWorldWrenchCmd(wrenchMsg));
      }
  }

  void ForcePlugin::OnWrenchMsg(const geometry_msgs::msg::Wrench::SharedPtr msg)
  {
    // Store force and torque directly in Vector3d
    force_.Set(msg->force.x, msg->force.y, msg->force.z);
    torque_.Set(msg->torque.x, msg->torque.y, msg->torque.z);
  }
}

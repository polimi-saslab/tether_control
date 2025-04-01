#include "tether_control/apply_force_plugin.hpp"

namespace gz
{
  namespace sim
  {
    ApplyForcePlugin::ApplyForcePlugin() : node_(rclcpp::Node::make_shared("apply_force_plugin"))
    {
      // Initialize the ROS2 subscriber
      force_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
        "apply_force", 10, std::bind(&ApplyForcePlugin::forceCallback, this, std::placeholders::_1));
    }

    ApplyForcePlugin::~ApplyForcePlugin() { rclcpp::shutdown(); }

    void ApplyForcePlugin::Configure(const gz::sim::Entity &entity)
    {
      // Get the model from the entity
      auto model = gz::sim::Model(entity);
      link_ = model.LinkByName("link_name"); // Replace "link_name" with your link name

      if(!link_)
        {
          RCLCPP_ERROR(node_->get_logger(), "Link not found!");
          return;
        }

      RCLCPP_INFO(node_->get_logger(), "Plugin configured with link: %s", link_->Name().c_str());
    }

    void ApplyForcePlugin::Update(const gz::sim::UpdateInfo &info)
    {
      // ROS spinning and force application logic
      rclcpp::spin_some(node_);

      if(link_)
        {
          // Apply force logic here (force is updated from the ROS topic)
          // Assuming force_ is a member variable storing the force vector
          gz::math::Vector3d force(link_->WorldPose().Rot().RotateVector(force_));
          link_->AddForce(force);
        }
    }

    void ApplyForcePlugin::forceCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
      // Update the force vector from the ROS topic
      force_ = gz::math::Vector3d(msg->x, msg->y, msg->z);
    }

  } // namespace sim
} // namespace gz

// Register the plugin with Gazebo
// GZ_ADD_PLUGIN(sample_system::ApplyForcePlugin, gz::sim::System)

GZ_REGISTER_SYSTEM_PLUGIN(ApplyForcePlugin)

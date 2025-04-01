#include "tether_control/apply_force_plugin.hpp"

namespace gz
{
  namespace sim
  {
    ApplyForcePlugin::ApplyForcePlugin() : node_(nullptr) {}

    ApplyForcePlugin::~ApplyForcePlugin() {}

    void ApplyForcePlugin::Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf)
    {
      // Initialize ROS 2 node
      if(!rclcpp::ok())
        {
          rclcpp::init(0, nullptr);
        }

      node_ = rclcpp::Node::make_shared("apply_force_plugin");

      // Initialize Gazebo transport node
      gz_node_.Init();

      // Store the entity to apply forces to it
      entity_ = entity;

      // Subscribe to the ROS topic
      force_subscriber_ = gz_node_.Subscribe("/apply_force", &ApplyForcePlugin::ApplyForceCallback, this);

      RCLCPP_INFO(node_->get_logger(), "ApplyForcePlugin initialized");
    }

    void ApplyForcePlugin::ApplyForceCallback(const geometry_msgs::msg::Wrench::SharedPtr msg)
    {
      // Apply force and torque to the drone's link in Gazebo
      if(entity_ != gz::sim::kNullEntity)
        {
          auto model = gz::sim::Model(entity_);
          if(model.Valid())
            {
              // Apply force to the model's link
              model.ApplyForce(gz::math::Vector3d(msg->force.x, msg->force.y, msg->force.z),
                               gz::math::Vector3d(msg->torque.x, msg->torque.y, msg->torque.z));
              RCLCPP_INFO(node_->get_logger(), "Applied force: (%f, %f, %f)", msg->force.x, msg->force.y, msg->force.z);
            }
        }
    }

  } // namespace sim
} // namespace gz

// Register the plugin with Gazebo
GZ_ADD_PLUGIN(sample_system::ApplyForcePlugin, gz::sim::System)

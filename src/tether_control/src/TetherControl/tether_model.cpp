#include "tether_control/tether_control.hpp"

using namespace std::chrono;

namespace tether_control
{
  void TetherControl::publishTetherForceDisturbations()
  {
    geometry_msgs::msg::WrenchStamped msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = "map";

    if(this->disturb_mode == DisturbationMode::CIRCULAR)
      {
        static double theta = 0.0; // angle for circular motion
        double radius = 1.5;       // radius of the circle (i.e., force magnitude in XY)
        double d_theta = 0.01;     // increment angle each call (controls speed)

        msg.wrench.force.x = radius * std::cos(theta);
        msg.wrench.force.y = radius * std::sin(theta);
        msg.wrench.force.z = -5.0; // Constant downward force
        msg.wrench.torque.x = 0.0;
        msg.wrench.torque.y = 0.0;
        msg.wrench.torque.z = 0.0;

        theta += d_theta;
        if(theta > 2 * M_PI)
          theta -= 2 * M_PI;
      }
    else if(this->disturb_mode == DisturbationMode::STRONG_SIDE)
      {
        msg.wrench.force.x = -2.5;
        msg.wrench.force.y = -2.5;
        msg.wrench.force.z = -2.5;
        msg.wrench.torque.x = 0.0;
        msg.wrench.torque.y = 0.0;
        msg.wrench.torque.z = 0.0;
      }
    else if(this->disturb_mode == DisturbationMode::TET_GRAV_FIL_ANG)
      {
        double resulting_force = 0.0;
        double delta_theta = 0.0;
        tether_model_.computeTetherVecForce(&resulting_force, &delta_theta);

        // assuming spherical coordinates for ENU frame:, inversing to create reaction force to the drone
        msg.wrench.force.x = -(resulting_force)*std::sin(delta_theta + this->tether_ground_cur_angle_theta)
                             * std::cos(this->tether_ground_cur_angle_phi);
        msg.wrench.force.y = -(resulting_force)*std::sin(delta_theta + this->tether_ground_cur_angle_theta)
                             * std::sin(this->tether_ground_cur_angle_phi);
        msg.wrench.force.z = -(esulting_force)*std::cos(
          delta_theta + this->tether_ground_cur_angle_theta); // should be pi - theta, then z=-z but same
      }
    else
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), LOG_THROT_FREQ, "Disturbation mode not defined");
      }

    // sanity check
    if(std::isnan(msg.wrench.force.x) || std::isnan(msg.wrench.force.y) || std::isnan(msg.wrench.force.z))
      {
        RCLCPP_WARN(this->get_logger(), "Nan values in msg force, sanitizing to 0");
        msg.wrench.force.x = 0.0;
        msg.wrench.force.y = 0.0;
        msg.wrench.force.z = 0.0;
        msg.wrench.torque.x = 0.0;
        msg.wrench.torque.y = 0.0;
        msg.wrench.torque.z = 0.0;
      }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROT_FREQ, "Publishing tether force in %f %f %f",
                         msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);

    this->tether_force_vec = {msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z};
    this->tether_force_pub_->publish(msg);
  }

  void TetherControl::setHoverThrust()
  {
    // Set the hover thrust based on the UAV type and tethered configuration
    if(this->uav_type == "tarot")
      {
        if(this->tethered)
          {
            this->hoverThrust = 0.3733f + 0.1f;
          }
        else
          {
            this->hoverThrust = 0.3733f; // # empirically determined
          }
      }
    else if(this->uav_type == "x500")
      {
        // random values, not sure if needed anyway
        if(this->tethered)
          {
            this->hoverThrust = 0.5;
          }
        else
          {
            this->hoverThrust = 0.4;
          }
      }
    else
      {
        RCLCPP_ERROR(this->get_logger(), "Unknown UAV type, hover thrust set to 0.5");
        this->hoverThrust = 0.5f;
      }
    RCLCPP_INFO(this->get_logger(), "Hover thrust set to: %f", this->hoverThrust);
  }

} // namespace tether_control

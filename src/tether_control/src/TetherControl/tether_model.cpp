#include "tether_control/tether_control.hpp"

using namespace std::chrono;

namespace tether_control
{
  void TetherControl::publishTetherForceDisturbations()
  {
    geometry_msgs::msg::WrenchStamped msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

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
        // Specific model for tether force
        this->tether_cur_length = this->dist_gs_drone; // easy way atm

        float tether_mass
          = this->tether_density * M_PI * std::pow(this->tether_diameter / 2, 2) * this->tether_cur_length; // [kg]
        float tether_ang_due_to_grav
          = 1 / 2
            * (tether_mass * this->gravComp * std::sin(this->tether_ground_cur_angle_theta)
               / this->winch_force); // == 1/2(M_t*g*cos(beta)/T), from eq (39) of The Influence of
        // Tether Sag on Airborne Wind Energy Generation by F. Trevisi
        // need to rotate vector from world to ENU, since publishing tether_force
        // in ENU frame

        this->tether_grav_force = tether_mass * this->gravComp; // [N] force on drone due to gravity of tether weight

        // assuming spherical coordinates for ENU frame:, inversing for drone pov
        msg.wrench.force.x = -(this->tether_grav_force + this->winch_force)
                             * std::sin(this->tether_ground_cur_angle_theta)
                             * std::cos(this->tether_ground_cur_angle_phi);
        msg.wrench.force.y = -(this->tether_grav_force + this->winch_force)
                             * std::sin(tether_ang_due_to_grav + this->tether_ground_cur_angle_theta)
                             * std::sin(this->tether_ground_cur_angle_phi);
        msg.wrench.force.z
          = -(tether_grav_force + this->winch_force)
            * std::cos(tether_ang_due_to_grav
                       + this->tether_ground_cur_angle_theta); // should be pi - theta, then z=-z but same
        RCLCPP_DEBUG(this->get_logger(),
                     "tether_cur_length: %f, tether_mass: %f, tether_ang_due_to_grav: %f, "
                     "tether_ground_cur_angle_theta: %f, tether_ground_cur_angle_phi: %f",
                     this->tether_cur_length, tether_mass, tether_ang_due_to_grav, this->tether_ground_cur_angle_theta,
                     this->tether_ground_cur_angle_phi);
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

    this->tether_force_pub_->publish(msg);
    if(this->debug_mode)
      {
        // inversion so that rviz makes sense
        geometry_msgs::msg::WrenchStamped msg_viz = msg;
        msg_viz.wrench.force.x = -msg.wrench.force.x;
        msg_viz.wrench.force.y = -msg.wrench.force.y;
        msg_viz.wrench.force.z = -msg.wrench.force.z;
        this->tether_force_viz_pub_->publish(msg_viz);
      }
  }

} // namespace tether_control

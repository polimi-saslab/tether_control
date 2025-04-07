#include "tether_control/tether_control.hpp"

////////////////////////////////////// Callback functions //////////////////////////////////////

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace tether_control
{
  void TetherControl::vehicleLocalPositionSubCb(const px4_msgs::msg::VehicleLocalPosition msg)
  {
    // storing local pos, to convert to lambda function if we don't do anything more with the sub
    this->local_pos_latest = msg;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Drone position: [%f, %f, %f]", msg.x, msg.y,
                         msg.z);
    if((msg.z <= -1.0) && (!this->is_init_pos))
      {
        RCLCPP_INFO_ONCE(this->get_logger(), "-------------- POSITION READY --------------");
        this->is_init_pos = true;
      }
    // if(((msg.x <= this->starting_pos[0] + 0.1) && (msg.x >= this->starting_pos[1] - 0.1))
    //    && ((msg.y <= this->starting_pos[1] + 0.1) && (msg.y >= this->starting_pos[1] - 0.1))
    //    && ((msg.z <= this->starting_pos[2] + 0.1) && (msg.z >= this->starting_pos[2] - 0.1)) && (!this->is_init_pos))
    //   {
    //     RCLCPP_INFO_ONCE(this->get_logger(), "-------------- POSITION READY --------------");
    //     this->is_init_pos = true;
    //   }
  }

  void TetherControl::vehicleStatusSubCb(const px4_msgs::msg::VehicleStatus msg)
  {
    bool last_status = false;
    this->is_node_alive = true; // -> means VehicleStatus topic is being published hence we have a drone alive
    if(msg.pre_flight_checks_pass)
      {
        RCLCPP_INFO_ONCE(this->get_logger(), "-------------- PREFLIGHT CHECKS PASSED --------------");
        this->prechecks_passed = true;
        last_status = true;
      }
    else if((last_status) && (!msg.pre_flight_checks_pass)) // should not happen theoretically
      {
        RCLCPP_INFO(this->get_logger(), "-------------- PREFLIGHT CHECKS NO LONGER PASS--------------");
        this->prechecks_passed = false;
        last_status = false;
      }
  }

  void TetherControl::vehicleTetherForceSubCb(const geometry_msgs::msg::Wrench msg)
  {
    // storing local pos, to convert to lambda function if we don't do anything more with the sub
    this->drone_tether_force_latest = msg;

    // plotting once just for info
    RCLCPP_INFO_ONCE(this->get_logger(), "-------------- GOT TETHER FORCE DATA --------------");
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Force: %f %f %f, Torque: %f %f %f", msg.force.x,
                         msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z);
  }

  void TetherControl::droneImuSubCb(const sensor_msgs::msg::Imu msg)
  {
    // storing local pos, to convert to lambda function if we don't do anything more with the sub
    this->drone_imu_latest = msg;

    // plotting once just for info
    RCLCPP_INFO_ONCE(this->get_logger(), "-------------- GOT IMU DATA --------------");
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "ACCEL Z IMU: %f ", msg.linear_acceleration.z);
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
    //  "Orientation: %f %f %f %f, angular velocity: %f %f %f, lin_accel: %f %f %f", msg.orientation.w,
    //  msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.angular_velocity.x,
    //  msg.angular_velocity.y, msg.angular_velocity.z, msg.linear_acceleration.x,
    //  msg.linear_acceleration.y, msg.linear_acceleration.z);
  }

} // namespace tether_control
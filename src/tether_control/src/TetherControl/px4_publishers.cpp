/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief First offboard control implementation for TO/LD control of tethered drone
 * @file px4_publishers.cpp
 * @author Yannis Coderey <yannis09@yahoo.fr>
 */

#include "tether_control/tether_control.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

////////////////////////////////////// PX4 publish functions //////////////////////////////////////

namespace tether_control
{

  /**
   * @brief Send a command to Arm the vehicle
   */
  void TetherControl::arm()
  {
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
  }

  /**
   * @brief Send a command to Disarm the vehicle
   */
  void TetherControl::disarm()
  {
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
  }

  /**
   * @brief Publish the offboard control mode.
   *        For this example, only position and altitude controls are active.
   */
  void TetherControl::publishOffboardControlMode(const std::vector<bool> &control_modes)
  {
    if(control_modes.size() != 5)
      {
        RCLCPP_ERROR(this->get_logger(), "Invalid control_modes vector size. Expected 5 elements.");
        return;
      }

    OffboardControlMode msg{};
    msg.position = control_modes[0];
    msg.velocity = control_modes[1];
    msg.acceleration = control_modes[2];
    msg.attitude = control_modes[3];
    msg.direct_actuator = control_modes[4];

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
  }

  /**
   * @brief Publish a trajectory setpoint
   *        For this example, it sends a trajectory setpoint to make the
   *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
   */
  void TetherControl::publishTrajectorySetpoint()
  {
    TrajectorySetpoint msg{};
    msg.position = this->starting_pos;
    msg.yaw = -3.14;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
  }

  void TetherControl::publishTrajectorySetpointCircle()
  {
    static float angle = 0.0f;        // radians
    static const float radius = 1.2f; // meters
    static const float step = 0.003f; // radians per call (adjust for speed)

    // Advance angle
    angle += step;
    if(angle > 2 * M_PI)
      {
        angle -= 2 * M_PI;
      }

    TrajectorySetpoint msg{};
    msg.position = {radius * cos(angle), radius * sin(angle), -1.0}; //{2.0, 2.0, -2.0};
    msg.yaw = -3.14;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), LOG_THROT_FREQ, "Publishing circles: [%f, %f, %f]",
                         msg.position[0], msg.position[1], msg.position[2]);
    trajectory_setpoint_publisher_->publish(msg);
  }

  /**
   * @brief Publish vehicle commands
   * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
   * @param param1    Command parameter 1
   * @param param2    Command parameter 2
   */
  void TetherControl::publishVehicleCommand(uint16_t command, float param1, float param2)
  {
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
  }

  void TetherControl::publishAttitudeSetpoint(const Eigen::Vector4d &controller_output,
                                              const Eigen::Quaterniond &desired_quat)
  {
    px4_msgs::msg::VehicleAttitudeSetpoint attitude_setpoint_msg;
    // Prepare AttitudeSetpoint msg;
    attitude_setpoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    Eigen::Quaterniond rotated_quat;
    rotated_quat = rotateQuaternionFromToENU_NED(desired_quat);
    attitude_setpoint_msg.q_d[0] = rotated_quat.w();
    attitude_setpoint_msg.q_d[1] = rotated_quat.x();
    attitude_setpoint_msg.q_d[2] = rotated_quat.y();
    attitude_setpoint_msg.q_d[3] = rotated_quat.z();

    if(controller_output[3] > 0.1)
      {
        attitude_setpoint_msg.thrust_body[0] = 0.0;
        attitude_setpoint_msg.thrust_body[1] = 0.0;
        attitude_setpoint_msg.thrust_body[2] = -controller_output[3]; // DO NOT FORGET THE MINUS SIGN (body NED frame)
      }
    else
      {
        attitude_setpoint_msg.thrust_body[2] = -0.1;
      }

    attitude_pub_->publish(attitude_setpoint_msg);
  }

} // namespace tether_control
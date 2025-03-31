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
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include "tether_control/offboard_control.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace offboard_control
{
  OffboardControl::OffboardControl() : Node("offboard_control")
  {
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    actuators_motors_pub = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors", 10);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    vehicle_status_sub = this->create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos, std::bind(&OffboardControl::vehicleStatusSubCb, this, std::placeholders::_1));

    vehicle_local_position_sub = this->create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos,
      std::bind(&OffboardControl::vehicleLocalPositionSubCb, this, std::placeholders::_1));

    offboard_setpoint_counter_ = 0;

    auto alive_timer_callback = [this]() -> void {
      if(this->isNodeAlive)
        {
          RCLCPP_INFO_ONCE(this->get_logger(), "NODE ALIVE");
          return;
        }
      else
        {
          RCLCPP_INFO(this->get_logger(), "Drone not alive ...");
        }
    };

    auto timer_callback = [this]() -> void {
      if(!this->isNodeAlive)
        {
          return;
        }
      return;
      if(!this->isPosRdy)
        {
          RCLCPP_INFO(this->get_logger(), "Going to initial position");
          publish_offboard_control_mode({true, false, false, false, false});
          publish_trajectory_setpoint();
        }
      else
        {
          RCLCPP_INFO(this->get_logger(), "Controller phase");
        }
      // update_motors();
      // doesn't work when already armed, for iterative commands
      if(!this->isArmed && this->preChecksPassed)
        {
          // Change to Offboard mode after 10 setpoints
          this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

          // Arm the vehicle
          this->arm();
          this->isArmed = true;
        }

      // stop the counter after reaching 11
      if(offboard_setpoint_counter_ < 11)
        {
          offboard_setpoint_counter_++;
        }
    };
    timer_ = this->create_wall_timer(10ms, timer_callback);
    alive_timer_ = this->create_wall_timer(1000ms, alive_timer_callback);
  }

  /**
   * @brief Send a command to Arm the vehicle
   */
  void OffboardControl::arm()
  {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
  }

  /**
   * @brief Send a command to Disarm the vehicle
   */
  void OffboardControl::disarm()
  {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
  }

  /**
   * @brief Publish the offboard control mode.
   *        For this example, only position and altitude controls are active.
   */
  void OffboardControl::publish_offboard_control_mode(const std::vector<bool> &control_modes)
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
  void OffboardControl::publish_trajectory_setpoint()
  {
    TrajectorySetpoint msg{};
    msg.position = {0.0, 0.0, -5.0};
    msg.yaw = -3.14; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
  }

  /**
   * @brief Publish vehicle commands
   * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
   * @param param1    Command parameter 1
   * @param param2    Command parameter 2
   */
  void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
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

  /**
   * @brief Publish vehicle commands
   * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
   * @param param1    Command parameter 1
   * @param param2    Command parameter 2
   */
  void OffboardControl::update_motors(const Eigen::Matrix<float, kMaxNumMotors, 1> &motor_commands)
  {
    px4_msgs::msg::ActuatorMotors act_motors{};
    for(int i = 0; i < kMaxNumMotors; ++i)
      {
        act_motors.control[i] = motor_commands(i);
      }
    act_motors.timestamp = 0; // Let PX4 set the timestamp
    act_motors.reversible_flags = 0;

    actuators_motors_pub->publish(act_motors);
  }

  /**
   * @brief Take-off controller
   * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
   * @param param1    Command parameter 1
   * @param param2    Command parameter 2
   */
  void OffboardControl::to_controller()
  {
    auto msg = px4_msgs::msg::ActuatorMotors();
    msg.timestamp = this->now().nanoseconds() / 1000; // Convert to microseconds
    msg.control = {0.5,
                   0.1,
                   0.1,
                   0.1,
                   std::nanf("1"),
                   std::nanf("1"),
                   std::nanf("1"),
                   std::nanf("1"),
                   std::nanf("1"),
                   std::nanf("1"),
                   std::nanf("1"),
                   std::nanf("1")};
    msg.reversible_flags = 0;
  }

  void OffboardControl::vehicleStatusSubCb(const px4_msgs::msg::VehicleStatus msg)
  {
    bool last_status = false;
    if(msg.pre_flight_checks_pass)
      {
        RCLCPP_INFO_ONCE(this->get_logger(), "-------------- Pre-flight checks passed --------------");
        this->preChecksPassed = true;
        last_status = true;
      }
    else if((last_status) && (!msg.pre_flight_checks_pass)) // should not happen theoretically
      {
        RCLCPP_INFO(this->get_logger(), "PREFLIGHT CHECKS NO LONGER PASS");
        this->preChecksPassed = false;
        last_status = false;
      }
  }

  void OffboardControl::vehicleLocalPositionSubCb(const px4_msgs::msg::VehicleLocalPosition msg)
  {
    // storing local pos, to convert to lambda function if we don't do anything more with the sub
    this->local_pos_latest = msg;
    if(msg.z == this->starting_pos[2])
      {
        this->isPosRdy = true;
      }
  }

} // namespace offboard_control

int main(int argc, char *argv[])
{
  std::cout << "Starting offboard control node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<offboard_control::OffboardControl>());

  rclcpp::shutdown();
  return 0;
}
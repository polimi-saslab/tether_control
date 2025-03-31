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

#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <tether_control/control.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace offboard_control
{
  class OffboardControl : public rclcpp::Node
  {
  public:
    explicit OffboardControl();

    static constexpr int kMaxNumMotors = px4_msgs::msg::ActuatorMotors::NUM_CONTROLS;

    void arm();
    void disarm();

    enum ControlMode
    {
      POSITION_CONTROL = 0,
      VELOCITY_CONTROL = 1,
      ACCELERATION_CONTROL = 2,
      ATTITUDE_CONTROL = 3,
      DIRECT_ACTUATORS = 4
    };

  private:
    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr alive_timer_;

    // Condition variables
    bool isArmed = false;
    bool preChecksPassed = false;
    bool isPosRdy = false;
    bool isNodeAlive = false;
    uint8_t controlMode = ControlMode::ATTITUDE_CONTROL; // default to attitude control for the moment

    // PX4 subscription data
    VehicleLocalPosition local_pos_latest;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<ActuatorMotors>::SharedPtr actuators_motors_pub;

    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_sub;

    std::atomic<uint64_t> timestamp_; //!< common synced timestamped

    uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent

    void publishOffboardControlMode(const std::vector<bool> &control_modes);
    void publishTrajectorySetpoint();
    void publishAttitudeSetpoint();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void
    publishAttitudeSetpointMsg(const Eigen::Vector4d &controller_output, const Eigen::Quaterniond &desired_quaternion);

    void vehicleStatusSubCb(const px4_msgs::msg::VehicleStatus msg);
    void vehicleLocalPositionSubCb(const px4_msgs::msg::VehicleLocalPosition msg);

    void updateMotors(const Eigen::Matrix<float, kMaxNumMotors, 1> &motor_commands);
    void takeOffController();

    std::vector<bool> position_control = {true, false, false, false, false};
    std::vector<bool> direct_actuator_control = {false, false, false, false, true};
    std::vector<float> starting_pos = {0.0f, 0.0f, -1.0f};
  };

} // namespace offboard_control
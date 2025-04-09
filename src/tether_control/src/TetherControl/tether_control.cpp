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
 * @file tether_control.cpp
 * @author Yannis Coderey <yannis09@yahoo.fr>
 */

#include "tether_control/tether_control.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace tether_control
{
  TetherControl::TetherControl(const std::string &nodeName) : Node(nodeName)
  {
    RCLCPP_INFO(this->get_logger(), "################# TETHER CONTROL NODE INITIALIZING #################");

    std::string control_mode_s;

    // Init parameters
    this->uav_type = this->declare_parameter<std::string>("uav_type", "MC");
    this->tethered = this->declare_parameter<bool>("tethered", true);
    control_mode_s = this->declare_parameter<std::string>("control_mode", "ATTITUDE");
    this->hoverThrust = this->declare_parameter<float>("hoverThrust", 0.7f);
    this->gravComp = this->declare_parameter<float>("gravComp", 9.81f);
    this->attThrustKp = this->declare_parameter<float>("attThrustKp", 0.5f);
    this->attThrustKd = this->declare_parameter<float>("attThrustKd", 0.05f);

    RCLCPP_INFO(this->get_logger(), "------------------- PARAMETERS --------------------");
    RCLCPP_INFO(this->get_logger(), "control_mode: %s", control_mode_s.c_str());
    RCLCPP_INFO(this->get_logger(), "uav_type: %s", this->uav_type.c_str());
    RCLCPP_INFO(this->get_logger(), "tethered: %i", this->tethered);
    RCLCPP_INFO(this->get_logger(), "hoverThrust: %f", this->hoverThrust);
    RCLCPP_INFO(this->get_logger(), "gravComp: %f", this->gravComp);
    RCLCPP_INFO(this->get_logger(), "attThrustKp: %f", this->attThrustKp);
    RCLCPP_INFO(this->get_logger(), "attThrustKd: %f", this->attThrustKd);
    RCLCPP_INFO(this->get_logger(), "---------------------------------------------------");

    convertControlMode(control_mode_s);

    // Init publishers
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    actuators_motors_pub = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors", 10);
    attitude_pub_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
    tether_force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/drone/tether_force", 10);

    // PX4 requires specific QoS, see
    // https://docs.px4.io/main/en/ros2/user_guide.html#compatibility-issues
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Init subscribers
    vehicle_status_sub = this->create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos, std::bind(&TetherControl::vehicleStatusSubCb, this, std::placeholders::_1));
    vehicle_local_position_sub = this->create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos,
      std::bind(&TetherControl::vehicleLocalPositionSubCb, this, std::placeholders::_1));
    vehicle_tether_force_sub = this->create_subscription<geometry_msgs::msg::Wrench>(
      "/tether_lin/base_link/ForceTorque", qos,
      std::bind(&TetherControl::vehicleTetherForceSubCb, this, std::placeholders::_1));
    drone_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "/tether/imu0", qos, std::bind(&TetherControl::droneImuSubCb, this, std::placeholders::_1));

    auto alive_timer_callback = [this]() -> void {
      if(this->is_node_alive)
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
      if(!this->is_node_alive)
        {
          return;
        }
      if(!this->is_armed)
        {
          if(this->prechecks_passed)
            {
              this->publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // Offboard mode
              this->arm();
              this->is_armed = true;
            }
          else
            {
              RCLCPP_WARN(this->get_logger(), "Prechecks not passed yet");
              return;
            }
        }

      // drone armed
      if(!this->is_init_pos) // is alive, armed but not in position
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROT_FREQ, "Control in mode INIT");
          publishOffboardControlMode({true, false, false, false, false});
          publishTrajectorySetpoint();
        }
      else if(this->control_mode == ControlMode::POSITION)
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROT_FREQ, "Control in mode POSITION");
          publishOffboardControlMode({true, false, false, false, false});
          publishTrajectorySetpoint();
        }
      else if(this->control_mode == ControlMode::DIRECT_ACTUATORS)
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROT_FREQ,
                               "Control in mode DIRECT_ACTUATORS");
          publishOffboardControlMode({false, false, false, false, true});
          updateMotors({this->hoverThrust, this->hoverThrust, this->hoverThrust, this->hoverThrust, std::nanf("1"),
                        std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"),
                        std::nanf("1")});
        }
      else if(this->control_mode == ControlMode::ATTITUDE)
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROT_FREQ, "Control in mode ATTITUDE");

          Eigen::Vector4d controller_output;
          controller_output[3] = 0.4f; // thrust
          Eigen::Quaterniond desired_quat = Eigen::Quaterniond::Identity();

          // pidController(controller_output, desired_quat);
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROT_FREQ,
                               "Controller output: %f, %f, %f, %f, desired quat: %f, %f, %f, %f", controller_output[0],
                               controller_output[1], controller_output[2], controller_output[3], desired_quat.w(),
                               desired_quat.x(), desired_quat.y(), desired_quat.z());
          publishOffboardControlMode({false, false, false, true, false});
          publishAttitudeSetpoint(controller_output, desired_quat);
        }
      else if(this->control_mode == ControlMode::TETHER_FORCE_REACTIONS)
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROT_FREQ,
                               "Control in mode TETHER_FORCE_REACTIONS");
          publishOffboardControlMode({true, false, false, false, false});
          publishTrajectorySetpointCircle();
        }
      else
        {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROT_FREQ,
                                "Unknown control mode, not doing anything...");
          this->control_mode = ControlMode::ATTITUDE;
          publishOffboardControlMode({false, false, false, true, false});
          publishAttitudeSetpoint(Eigen::Vector4d::Zero(), Eigen::Quaterniond::Identity());
        }
    };
    timer_ = this->create_wall_timer(10ms, timer_callback);
    alive_timer_ = this->create_wall_timer(1000ms, alive_timer_callback);

    RCLCPP_INFO(this->get_logger(), "################# TETHER CONTROL NODE INITIALIZED #################");
  }

  ////////////////////////////////////// Control functions //////////////////////////////////////
  void TetherControl::updateMotors(const Eigen::Matrix<float, kMaxNumMotors, 1> &motor_commands)
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

  // @todo: create class for PID controller, cleaner
  void TetherControl::pidController(Eigen::Vector4d &controller_output) //, Eigen::Quaterniond &desired_quat
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "Got tether force sensor data");

    sensor_msgs::msg::Imu imu_msg = this->drone_imu_latest;
    // Extract the orientation quaternion
    geometry_msgs::msg::Quaternion orientation = imu_msg.orientation;
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    // Convert linear acceleration to tf2 Vector3
    tf2::Vector3 accel_body(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
                            imu_msg.linear_acceleration.z);
    // Rotate to world frame
    tf2::Matrix3x3 R(q);
    tf2::Vector3 accel_world = R * accel_body;
    float t_s = 0.01f;

    // geometry_msgs::msg::Quaternion orientation = imu_msg.orientation;
    float cur_accel_z_world = accel_world.getZ();
    float cur_er_accel_z = this->gravComp - cur_accel_z_world; // gz imu includes gravity
    if(this->last_er_accel_z == 0.0f)                          // first time we get the data
      {
        this->last_er_accel_z = cur_er_accel_z;
      }

    float cur_er_d_accel_z = (cur_er_accel_z - this->last_er_accel_z) / t_s; // derivative of error

    float max_adjustment = 0.15f; // how much you allow PID to push
    float kp_adjustment = std::clamp(this->attThrustKp * cur_er_accel_z, -max_adjustment, max_adjustment);
    float kd_adjustment = std::clamp(this->attThrustKd * cur_er_d_accel_z, -max_adjustment, max_adjustment);
    float thrust = this->hoverThrust + kp_adjustment + kd_adjustment;
    thrust = std::clamp(thrust, 0.0f, 1.0f); // normalize it
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Accel z error: %f, Kp adjustment: %f, Thrust %f",
                         cur_er_accel_z, kp_adjustment, thrust);

    controller_output[3] = thrust;

    this->last_er_accel_z = cur_er_accel_z; // update error
  }

  void TetherControl::publishTetherForceDisturbations()
  {
    geometry_msgs::msg::WrenchStamped msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    switch(counter_ % 4)
      {
      case 0:
        msg.wrench.force.x = -7.0;
        msg.wrench.force.y = 0.0;
        msg.wrench.force.z = -3.0;
        RCLCPP_INFO_ONCE(this->get_logger(), "Publishing tether force in %f %f %f", msg.wrench.force.x,
                         msg.wrench.force.y, msg.wrench.force.z);
        counter_time++;
        break;
      case 1:
        msg.wrench.force.x = 0.0;
        msg.wrench.force.y = -5.0;
        msg.wrench.force.z = -2.0;
        RCLCPP_INFO_ONCE(this->get_logger(), "Publishing tether force in %f %f %f", msg.wrench.force.x,
                         msg.wrench.force.y, msg.wrench.force.z);
        counter_time++;
        break;
      case 2:
        msg.wrench.force.x = 0.0;
        msg.wrench.force.y = 0.0;
        msg.wrench.force.z = -3.0;
        RCLCPP_INFO_ONCE(this->get_logger(), "Publishing tether force in %f %f %f", msg.wrench.force.x,
                         msg.wrench.force.y, msg.wrench.force.z);
        counter_time++;
        break;
      case 3:
        msg.wrench.force.x = -4.0;
        msg.wrench.force.y = -4.0;
        msg.wrench.force.z = -4.0;
        RCLCPP_INFO_ONCE(this->get_logger(), "Publishing tether force in %f %f %f", msg.wrench.force.x,
                         msg.wrench.force.y, msg.wrench.force.z);
        counter_time++;
        break;
      }
    if((counter_time % 500) == 0)
      counter_++;
    tether_force_pub_->publish(msg);
  }

} // namespace tether_control
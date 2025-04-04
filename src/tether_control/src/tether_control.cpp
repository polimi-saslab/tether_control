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

    // Init parameters
    this->hoverThrust = this->declare_parameter<float>("hoverThrust", 0.73f);
    this->attThrustKp = this->declare_parameter<float>("attThrustKp", 0.5f);
    this->attThrustKd = this->declare_parameter<float>("attThrustKd", 0.05f);

    RCLCPP_INFO(this->get_logger(), "------------------- PARAMETERS --------------------");
    RCLCPP_INFO(this->get_logger(), "hoverThrust value: %f", this->hoverThrust);
    RCLCPP_INFO(this->get_logger(), "Kp value: %f", this->attThrustKp);
    RCLCPP_INFO(this->get_logger(), "Kd value: %f", this->attThrustKd);
    RCLCPP_INFO(this->get_logger(), "---------------------------------------------------");

    // Init publishers
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    actuators_motors_pub = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors", 10);
    attitude_pub_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);

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
      "/tether/drone_joint/ForceTorque", qos,
      std::bind(&TetherControl::vehicleTetherForceSubCb, this, std::placeholders::_1));
    drone_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "/drone/imu0", qos, std::bind(&TetherControl::droneImuSubCb, this, std::placeholders::_1));

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
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROTTLE_FREQ, "Going to initial position");
          publishOffboardControlMode({true, false, false, false, false});
          publishTrajectorySetpoint();
        }
      else if(this->controlMode == ControlMode::DIRECT_ACTUATORS)
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROTTLE_FREQ, "Control in mode %d",
                               this->controlMode);
          publishOffboardControlMode({false, false, false, false, true});
          // 0.75 is T/O value for x500
          updateMotors({0.75, 0.75, 0.75, 0.75, std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"),
                        std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1")});
        }
      else if(this->controlMode == ControlMode::ATTITUDE_CONTROL)
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROTTLE_FREQ, "Control in mode %d",
                               this->controlMode);

          Eigen::Vector4d controller_output;
          Eigen::Quaterniond desired_quat = Eigen::Quaterniond::Identity();

          pid_controller(controller_output, desired_quat);
          publishOffboardControlMode({false, false, false, true, false});
          publishAttitudeSetpoint(controller_output, desired_quat);
        }
      else
        {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROTTLE_FREQ,
                                "Unknown control mode, not doing anything...");
          this->controlMode = ControlMode::ATTITUDE_CONTROL;
          publishOffboardControlMode({false, false, false, true, false});
          publishAttitudeSetpoint(Eigen::Vector4d::Zero(), Eigen::Quaterniond::Identity());
        }
    };
    timer_ = this->create_wall_timer(10ms, timer_callback);
    alive_timer_ = this->create_wall_timer(1000ms, alive_timer_callback);

    RCLCPP_INFO(this->get_logger(), "################# TETHER CONTROL NODE INITIALIZED #################");
  }

  ////////////////////////////////////// PX4 publish functions //////////////////////////////////////

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
    msg.position = {0.0, 0.0, -1.0};
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

  ////////////////////////////////////// Callback functions //////////////////////////////////////

  void TetherControl::vehicleLocalPositionSubCb(const px4_msgs::msg::VehicleLocalPosition msg)
  {
    // storing local pos, to convert to lambda function if we don't do anything more with the sub
    this->local_pos_latest = msg;
    if(((msg.z <= this->starting_pos[2] + 0.1) && (msg.z >= this->starting_pos[2] - 0.1)) && (!this->is_init_pos))
      {
        RCLCPP_INFO_ONCE(this->get_logger(), "-------------- POSITION READY --------------");
        RCLCPP_INFO(this->get_logger(), "Drone position: %f %f %f", msg.x, msg.y, msg.z);
        this->is_init_pos = true;
      }
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
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Orientation: %f %f %f %f, angular velocity: %f %f %f, lin_accel: %f %f %f", msg.orientation.w,
                         msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.angular_velocity.x,
                         msg.angular_velocity.y, msg.angular_velocity.z, msg.linear_acceleration.x,
                         msg.linear_acceleration.y, msg.linear_acceleration.z);
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

  void TetherControl::pid_controller(Eigen::Vector4d &controller_output, Eigen::Quaterniond &desired_quat)
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "Got tether force sensor data");
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Controller output: %f, %f, %f, %f, desired quat: %f, %f, %f, %f", controller_output[0],
                         controller_output[1], controller_output[2], controller_output[3], desired_quat.w(),
                         desired_quat.x(), desired_quat.y(), desired_quat.z());

    sensor_msgs::msg::Imu imu_msg = this->drone_imu_latest;

    // geometry_msgs::msg::Quaternion orientation = imu_msg.orientation;
    float cur_accel_z = imu_msg.linear_acceleration.z;
    float er_accel_z = this->hoverThrust - cur_accel_z; // gz imu includes gravity
    float er_d_accel_z = 11.0f - cur_accel_z;           // gz imu includes gravity

    float base_thrust = 0.7f;    // estimated hover thrust
    float max_adjustment = 0.3f; // how much you allow PID to push
    float kpAdjustment = std::clamp(this->attThrustKp * er_accel_z, -max_adjustment, max_adjustment);
    float kdAdjustment = std::clamp(this->attThrustKd * er_d_accel_z, -max_adjustment, max_adjustment);
    float thrust = base_thrust + adjustment;
    thrust = std::clamp(thrust, 0.0f, 1.0f); // normalize it
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "Accel z error: %f, Thrust adjustment: %f, Thrust %f", er_accel_z, adjustment, thrust);

    controller_output[3] = thrust;
  }

} // namespace tether_control
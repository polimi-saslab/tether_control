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

using namespace px4_msgs::msg;

namespace tether_control
{
  TetherControl::TetherControl(const std::string &nodeName) : Node(nodeName)
  {
    RCLCPP_INFO(this->get_logger(), "################# TETHER CONTROL NODE INITIALIZING #################");

    std::string control_mode_s;

    // Init parameters
    loadParams();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Init publishers
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    actuators_motors_pub = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors", 10);
    attitude_pub_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
    tether_force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/drone/tether_force", 10);
    tether_force_viz_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/drone/tether_force_viz", 10);
    tether_force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/drone/tether_force", 10);
    tether_model_metrics_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/metrics/model", 10);
    drone_rpy_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/drone/rpy", 10);

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
    vehicle_attitude_sub = this->create_subscription<VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos, std::bind(&TetherControl::vehicleAttitudeSubCb, this, std::placeholders::_1));
    winch_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/winch/joint_state", qos, std::bind(&TetherControl::winchJointStateSubCb, this, std::placeholders::_1));

    callback_group_main = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_log = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TetherControl::timer_callback, this),
                                     callback_group_main);
    timer_alive = this->create_wall_timer(std::chrono::milliseconds(1000),
                                          std::bind(&TetherControl::timer_alive_callback, this), callback_group_main);
    timer_log = this->create_wall_timer(std::chrono::milliseconds(100),
                                        std::bind(&TetherControl::timer_log_callback, this), callback_group_log);

    RCLCPP_INFO(this->get_logger(), "################# TETHER CONTROL NODE INITIALIZED #################");
  }

  void TetherControl::timer_alive_callback()
  {
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

  void TetherControl::timer_callback()
  {
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
            // @todo: add condition that checks if was armed successfully, otherwise don't put is_armed to true
          }
        else
          {
            RCLCPP_WARN(this->get_logger(), "Prechecks not passed yet");
            return;
          }
      }

    // transformation map -> base_link, mostly for visualization purposes
    if(this->debug_mode) // @todo: replace with ros parameter, i.e rviz_on
      {
        transformMapDrone();
      }

    // drone armed
    if(!this->is_init_pos) // is alive, armed but not in position
      {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROT_FREQ, "Control in mode INIT");
        publishOffboardControlMode({true, false, false, false, false});
        publishTrajectorySetpoint();
        return;
      }

    if(this->control_mode == ControlMode::POSITION)
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

        Eigen::Vector4d controller_output = {0.0, 0.0, 0.0, 0.0};
        // if we want to modify the thrust: (isnt the default one, see setHoverThrust func)
        if(this->debug_mode)
          this->hoverThrust = this->get_parameter("control.hoverThrust").as_double();
        controller_output[3] = this->hoverThrust; // thrust
        double roll_rad = this->get_parameter("control.attR").as_double() * M_PI / 180.0;
        double pitch_rad = this->get_parameter("control.attP").as_double() * M_PI / 180.0;
        double yaw_rad = this->get_parameter("control.attY").as_double() * M_PI / 180.0;

        // Convert to quaternion
        Eigen::Quaterniond desired_quat
          = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(roll_rad, pitch_rad, yaw_rad);
        // Eigen::Quaterniond desired_quat = Eigen::Quaterniond::Identity();

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
    else if(this->control_mode == ControlMode::CUSTOM)
      {
        Eigen::Vector4d controller_output = {0.0, 0.0, 0.0, 0.0};
        this->hoverThrust = this->get_parameter("control.hoverThrust").as_double();
        controller_output[3] = this->hoverThrust;
        Eigen::Quaterniond desired_quat = Eigen::Quaterniond::Identity(); // safe default

        forceCompensation(controller_output, desired_quat);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROT_FREQ, "Control in mode CUSTOM");
        publishOffboardControlMode({false, false, false, true, false});
        publishAttitudeSetpoint(controller_output, desired_quat);
      }
    else
      {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROT_FREQ,
                              "Unknown control mode, not doing anything...");
        this->control_mode = ControlMode::ATTITUDE;
        publishOffboardControlMode({false, false, false, true, false});
        publishAttitudeSetpoint(Eigen::Vector4d::Zero(), Eigen::Quaterniond::Identity());
      }

    if(this->tethered)
      {
        publishTetherForceDisturbations();
      }
  }

  // timer handling the publishers of sensor data, for analysis
  void TetherControl::timer_log_callback()
  {
    if(!this->log_mode)
      {
        return;
      }

    if(this->is_node_alive)
      {
        // do not abuse, may cause com problems!
        // to publish:
        // now: dist_gs_drone,tether_ground_cur_angle_theta, tether_ground_cur_angle_phi, tether_grav_force
        // when implemented: tether_cur_length, winch_force
        std_msgs::msg::Float32MultiArray msg_model;
        msg_model.data.resize(6);
        msg_model.data[0] = this->dist_gs_drone;                     // [m]
        msg_model.data[1] = this->tether_cur_length;                 // [m]
        msg_model.data[2] = this->tether_ground_cur_angle_theta;     // [rad]
        msg_model.data[3] = this->tether_ground_cur_angle_phi;       // [rad]
        msg_model.data[4] = this->winch_angle_latest * 180.0 / M_PI; // [rad]
        // msg_model.data[5] = this->tether_grav_force;                 // [N]

        tether_model_metrics_pub_->publish(msg_model);
        Eigen::Vector3d rpy_deg = quaternion_to_euler_deg(this->attitude_quat_latest);

        geometry_msgs::msg::Vector3 msg_rpy;
        msg_rpy.x = rpy_deg.x();
        msg_rpy.y = rpy_deg.y();
        msg_rpy.z = rpy_deg.z();
        drone_rpy_pub_->publish(msg_rpy);
      }
  }

} // namespace tether_control
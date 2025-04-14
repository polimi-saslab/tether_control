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
    // Control
    this->gravComp = this->declare_parameter<float>("gravComp", 9.81f);
    this->tethered = this->declare_parameter<bool>("tethered", true);
    this->debug_mode = this->declare_parameter<bool>("debug_mode", true);
    this->uav_type = this->declare_parameter<std::string>("uav_type", "MC");
    control_mode_s = this->declare_parameter<std::string>("control.control_mode", "ATTITUDE");
    this->hoverThrust = this->declare_parameter<float>("control.hoverThrust", 0.5f);
    this->attThrustKp = this->declare_parameter<float>("control.attThrustKp", 0.5f);
    this->attThrustKd = this->declare_parameter<float>("control.attThrustKd", 0.05f);
    this->attR = this->declare_parameter<float>("control.attR", 0.0f);
    this->attP = this->declare_parameter<float>("control.attP", 0.0f);
    this->attY = this->declare_parameter<float>("control.attY", 0.0f);
    // Model
    std::string disturb_mode_s = this->declare_parameter<std::string>("model.disturb_mode", "STRONG_SIDE");
    this->tether_init_length = this->declare_parameter<float>("model.tether_init_length", 1.0f);
    this->tether_diameter = this->declare_parameter<float>("model.tether_diameter", 0.005f);
    this->tether_density = this->declare_parameter<float>("model.tether_density", 970.0f);

    RCLCPP_INFO(this->get_logger(), "------------------- PARAMETERS --------------------");
    RCLCPP_INFO(this->get_logger(), "------------------- CONTROL --------------------");
    RCLCPP_INFO(this->get_logger(), "gravComp: %f", this->gravComp);
    RCLCPP_INFO(this->get_logger(), "tethered: %i", this->tethered);
    RCLCPP_INFO(this->get_logger(), "debug_mode: %i", this->debug_mode);
    RCLCPP_INFO(this->get_logger(), "uav_type: %s", this->uav_type.c_str());
    RCLCPP_INFO(this->get_logger(), "control_mode: %s", control_mode_s.c_str());
    RCLCPP_INFO(this->get_logger(), "hoverThrust: %f", this->hoverThrust);
    RCLCPP_INFO(this->get_logger(), "attThrustKp: %f", this->attThrustKp);
    RCLCPP_INFO(this->get_logger(), "attThrustKd: %f", this->attThrustKd);
    RCLCPP_INFO(this->get_logger(), "------------------- MODEL --------------------");
    RCLCPP_INFO(this->get_logger(), "disturb_mode: %s", disturb_mode_s.c_str());
    RCLCPP_INFO(this->get_logger(), "tether_init_length: %f", this->tether_init_length);
    RCLCPP_INFO(this->get_logger(), "tether_diameter: %f", this->tether_diameter);
    RCLCPP_INFO(this->get_logger(), "tether_density: %f", this->tether_density);
    RCLCPP_INFO(this->get_logger(), "---------------------------------------------------");

    convertControlMode(control_mode_s);
    convertDisturbMode(disturb_mode_s);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Init publishers
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    actuators_motors_pub = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors", 10);
    attitude_pub_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
    tether_force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/drone/tether_force", 10);
    if(this->debug_mode)
      tether_force_viz_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/drone/tether_force_viz", 10);

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
    vehicle_tether_force_sub = this->create_subscription<geometry_msgs::msg::Wrench>(
      "/tether_lin/base_link/ForceTorque", qos,
      std::bind(&TetherControl::vehicleTetherForceSubCb, this, std::placeholders::_1));
    drone_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "/tether/imu0", qos, std::bind(&TetherControl::droneImuSubCb, this, std::placeholders::_1));

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

        Eigen::Vector4d controller_output;
        this->hoverThrust = this->get_parameter("control.hoverThrust").as_double();
        controller_output[3] = this->hoverThrust; // thrust
        Eigen::Quaterniond desired_quat = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(
          this->get_parameter("control.attR").as_double(), this->get_parameter("control.attP").as_double(),
          this->get_parameter("control.attY").as_double()); // roll, pitch, yaw
        // Eigen::Quaterniond desired_quat = Eigen::Quaterniond::Identity();

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

    if(this->tethered)
      {
        if(this->disturb_mode == DisturbationMode::STRONG_SIDE)
          {
            publishTetherForceDisturbations();
          }
        else if(this->disturb_mode == DisturbationMode::CIRCULAR)
          {
            publishTetherForceDisturbations();
          }
        else if(this->disturb_mode == DisturbationMode::TET_GRAV_FIL_ANG)
          {
            publishTetherForceDisturbations();
          }
        else if(this->disturb_mode == DisturbationMode::NONE)
          {
            RCLCPP_WARN(this->get_logger(), "Unknown disturbation mode, defaulted to NONE");
          }
      }
  }

  void TetherControl::timer_log_callback()
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
  }

} // namespace tether_control
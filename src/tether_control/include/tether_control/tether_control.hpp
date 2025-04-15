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
 * @file tether_control.hpp
 * @author Yannis Coderey <yannis09@yahoo.fr>
 */

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Core>
#include <frame_transforms.h>
#include <geometry_msgs/msg/wrench.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <stdint.h>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

#define GPS_POS_ACCEPT_RANGE 0.1f // [m] acceptable range for GPS position
#define MC_HOVER_THRUST 0.73f     // [N] thrust to be applied to drone to hover, determined by simulation
#define X500_MASS = 2.0f + 4 * 0.016076923076923075f // [kg] mass of the drone + 4 motors
#define HOVER_FORCE = X500_MASS * 9.81f // [N] force to be applied to drone to hover, determined by simulation
#define LOG_THROT_FREQ 1000             // [Hz] frequency at which to log throttle values (normal)
#define LOG_THROT_FREQ_LOW 100          // [Hz] frequency at which to log throttle values (speedy)

namespace tether_control
{
  class TetherControl : public rclcpp::Node
  {
  public:
    explicit TetherControl(const std::string &nodeName);

    static constexpr int kMaxNumMotors = px4_msgs::msg::ActuatorMotors::NUM_CONTROLS; // 12

    void arm();
    void disarm();

    // available control modes
    enum class ControlMode
    {
      POSITION,
      ATTITUDE,
      DIRECT_ACTUATORS,
      TETHER_FORCE_REACTIONS,
      NONE
    };

    // model names, atm only TET_GRAV_FIL_ANG
    enum class DisturbationMode
    {
      NONE,
      STRONG_SIDE,
      CIRCULAR,
      TET_GRAV_FIL_ANG // tether grav force + angle according to Filippo's paper
    };

  private:
    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_alive;
    rclcpp::TimerBase::SharedPtr timer_log;

    void timer_log_callback();
    void timer_callback();
    void timer_alive_callback();

    rclcpp::CallbackGroup::SharedPtr callback_group_main;
    rclcpp::CallbackGroup::SharedPtr callback_group_log;

    // Generic condition variables
    std::string control_mode_s;
    bool is_armed = false;
    bool prechecks_passed = false;
    bool is_init_pos = false;
    bool is_node_alive = false;
    bool log_mode = true;   // whether we want to publish sensor data, to then analyze via ros2 bag
    bool debug_mode = true; // when true, enables rviz2 visualisation / transformations (force vector, etc)

    ////////////////// Parameters //////////////////
    // Control
    std::string uav_type = "MC"; // [MC, VTOL, VTOL_TAILSITTER]
    bool tethered = false;       // [true, false] true if the drone is tethered
    float hoverThrust = 0.37;    // [N] thrust to be applied to drone to hover, determined by simulation
    float gravComp = 9.81f;
    float attThrustKp = 0.5;
    float attThrustKd = 0.05;
    // Model
    float tether_density = 0.0f;     // [kg/m^3] density of the cable
    float tether_diameter = 0.1f;    // [m] radius of the cable
    float tether_init_length = 1.0f; // [m] length of the cable
    float winch_diameter = 0.1f;
    ///////////////////////////////////////////////

    ////////////////// Variables //////////////////
    // Control variables
    ControlMode control_mode = ControlMode::TETHER_FORCE_REACTIONS;
    std::array<float, 3> starting_pos = {0.0f, 0.0f, -2.0f};
    float last_er_accel_z = 0.0f;
    float attR, attP, attY; // roll, pitch, yaw
    // Model variables
    DisturbationMode disturb_mode = DisturbationMode::STRONG_SIDE;
    float winch_force = 1.5f;                // [N] tension force felt by the winch
    float dist_gs_drone = 0.0f;              // [m] distance between drone and ground station
    float tether_cur_length = dist_gs_drone; // [m] current length of the cable, assuming straight line atm
    float tether_drone_cur_angle = 0.0f;     // [rad] angle between the cable and the drone
    float tether_ground_cur_angle_theta;     // [rad] angle between the cable and the ground plane
    float tether_ground_cur_angle_phi;       // [rad] angle between the projection of the cable on ground and x

    // Sensor data
    float tether_grav_force;

    // PX4 subscription data
    Eigen::Quaterniond attitude_quat_latest;

    px4_msgs::msg::VehicleLocalPosition local_pos_latest; // @todo: just need to stock important var imo
    geometry_msgs::msg::Wrench drone_tether_force_latest;
    sensor_msgs::msg::Imu drone_imu_latest;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ROS2 Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuators_motors_pub;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr tether_force_pub_;
    // pub for visualisation / debugging purposes
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr tether_force_viz_pub_;
    // pub for analysing data
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr tether_model_metrics_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr drone_rpy_pub_;

    // Publish functions
    void publishOffboardControlMode(const std::vector<bool> &control_modes);
    void publishTrajectorySetpoint();
    void publishTrajectorySetpointCircle();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publishAttitudeSetpoint(const Eigen::Vector4d &controller_output, const Eigen::Quaterniond &desired_quat);
    void publishTetherForceDisturbations();

    // Susbcribers
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr vehicle_tether_force_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr drone_imu_sub;

    // Callback functions
    void droneImuSubCb(const sensor_msgs::msg::Imu msg);
    void vehicleAttitudeSubCb(const px4_msgs::msg::VehicleAttitude msg);
    void vehicleLocalPositionSubCb(const px4_msgs::msg::VehicleLocalPosition msg);
    void vehicleStatusSubCb(const px4_msgs::msg::VehicleStatus msg);
    void vehicleTetherForceSubCb(const geometry_msgs::msg::Wrench msg);

    // Controller functions
    void updateMotors(const Eigen::Matrix<float, kMaxNumMotors, 1> &motor_commands);
    void pidController(Eigen::Vector4d &controller_output); //, Eigen::Quaterniond &desired_quat

    // Utils
    Eigen::Quaterniond rotateQuaternionFromToENU_NED(const Eigen::Quaterniond &quat_in);
    double get_pitch_from_imu(const geometry_msgs::msg::Quaternion &quat);
    void convertControlMode(std::string control_mode_s);
    void convertDisturbMode(std::string disturb_mode_s);
    Eigen::Vector3d quaternion_to_euler_deg(const Eigen::Quaterniond &q);

    // Others
    std::atomic<uint64_t> timestamp_; //!< common synced timestamped
    void transformMapDrone();
  };

} // namespace tether_control
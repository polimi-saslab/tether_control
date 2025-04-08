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

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <Eigen/Core>
#include <geometry_msgs/msg/wrench.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <stdint.h>

#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

#define GPS_POS_ACCEPT_RANGE 0.1f // [m] acceptable range for GPS position
#define MC_HOVER_THRUST 0.73f     // [N] thrust to be applied to drone to hover, determined by simulation
#define X500_MASS = 2.0f + 4 * 0.016076923076923075f // [kg] mass of the drone + 4 motors
#define HOVER_FORCE = X500_MASS * 9.81f // [N] force to be applied to drone to hover, determined by simulation
#define LOG_THROTTLE_FREQ 2500          // [Hz] frequency at which to log throttle values

namespace tether_control
{
  class TetherControl : public rclcpp::Node
  {
  public:
    explicit TetherControl(const std::string &nodeName);

    static constexpr int kMaxNumMotors = px4_msgs::msg::ActuatorMotors::NUM_CONTROLS; // 12

    void arm();
    void disarm();

    enum ControlMode
    {
      POSITION_CONTROL = 0,
      VELOCITY_CONTROL = 1,
      ACCELERATION_CONTROL = 2,
      ATTITUDE_CONTROL = 3,
      DIRECT_ACTUATORS = 4,
      TETHER_FORCE_REACTIONS = 5
    };

  private:
    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr alive_timer_;

    // Condition variables
    bool is_armed = false;
    bool prechecks_passed = false;
    bool is_init_pos = false;
    bool is_node_alive = false;

    // Parameters
    float hoverThrust = 0.73f; // [N] thrust to be applied to drone to hover, determined by simulation
    float gravComp = 9.81f;
    float attThrustKp = 0.5;
    float attThrustKd = 0.05;

    // Random variables
    float droneHoverThrust = MC_HOVER_THRUST; // [N] thrust to be applied to drone to hover

    // Control variables
    uint8_t controlMode = ControlMode::TETHER_FORCE_REACTIONS;
    std::vector<bool> position_control = {true, false, false, false, false};
    std::vector<bool> direct_actuator_control = {false, false, false, false, true};
    std::vector<float> starting_pos = {0.0f, 0.0f, -1.0f};
    float last_er_accel_z = 0.0f;

    // variables for tether force reactions mode
    size_t counter_ = 0;
    size_t counter_time = 0;

    // PX4 subscription data
    px4_msgs::msg::VehicleLocalPosition local_pos_latest; // @todo: just need to stock important var imo
    geometry_msgs::msg::Wrench drone_tether_force_latest;
    sensor_msgs::msg::Imu drone_imu_latest;

    // ROS2 Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuators_motors_pub;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr tether_force_pub_;

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
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr vehicle_tether_force_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr drone_imu_sub;

    // Callback functions
    void droneImuSubCb(const sensor_msgs::msg::Imu msg);
    void vehicleLocalPositionSubCb(const px4_msgs::msg::VehicleLocalPosition msg);
    void vehicleStatusSubCb(const px4_msgs::msg::VehicleStatus msg);
    void vehicleTetherForceSubCb(const geometry_msgs::msg::Wrench msg);

    // Controller functions
    void updateMotors(const Eigen::Matrix<float, kMaxNumMotors, 1> &motor_commands);
    void pidController(Eigen::Vector4d &controller_output); //, Eigen::Quaterniond &desired_quat

    // Utils
    Eigen::Quaterniond rotateQuaternionFromToENU_NED(const Eigen::Quaterniond &quat_in);
    double get_pitch_from_imu(const geometry_msgs::msg::Quaternion &quat);

    // Others
    std::atomic<uint64_t> timestamp_; //!< common synced timestamped
  };

} // namespace tether_control
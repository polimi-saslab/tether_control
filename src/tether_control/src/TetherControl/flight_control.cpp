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

  void TetherControl::forceCompensation(Eigen::Vector4d &controller_output, Eigen::Quaterniond &desired_quat)
  {
    Eigen::Vector3d force_to_compensate = this->tether_force_vec + Eigen::Vector3d(0.0, 0.0, WEIGHT_TAROT);
    controller_output[3] = force_to_compensate.norm() / this->thrust_force_constant;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROT_FREQ,
                         "tether force: [%f, %f, %f], Force to compensate: [%f, %f, %f], thrust: %f",
                         this->tether_force_vec[0], this->tether_force_vec[1], this->tether_force_vec[2],
                         force_to_compensate[0], force_to_compensate[1], force_to_compensate[2], controller_output[3]);

    // compute quaternion to rotate drone s.t. its Z axis points towards it (NED frame)
    Eigen::Vector3d target_z = -force_to_compensate.normalized();
    Eigen::Vector3d body_z = -Eigen::Vector3d::UnitZ();

    double dot = body_z.dot(target_z);
    if(dot > 0.9999)
      {
        desired_quat = Eigen::Quaterniond::Identity(); // already aligned
      }
    else if(dot < -0.9999)
      {
        // Opposite direction: 180 deg rotation around X or Y
        desired_quat = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
      }
    else
      {
        Eigen::Vector3d axis = body_z.cross(target_z).normalized();
        double angle = std::acos(dot);
        desired_quat = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
      }
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
    float cur_er_accel_z = GRAVITY_FORCE - cur_accel_z_world; // gz imu includes gravity
    if(this->last_er_accel_z == 0.0f)                         // first time we get the data
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

} // namespace tether_control
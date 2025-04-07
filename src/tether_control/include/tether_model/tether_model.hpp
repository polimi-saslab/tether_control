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
 * @brief Modeling tether cable dynamics
 * @file tether_model.hpp
 * @author Yannis Coderey <yannis09@yahoo.fr>
 */

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

#define LOG_THROTTLE 1000

namespace tether_model
{
  class TetherModel : public rclcpp::Node
  {
  public:
    explicit TetherModel(const std::string &nodeName);

    enum class DisturbationMode
    {
      NONE,
      STRONG_SIDE,
      CIRCULAR,
      CUSTOM
    };

  private:
    // Timers
    rclcpp::TimerBase::SharedPtr timer_alive_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Model parameters
    float tether_density = 0.0f;     // [kg/m^3] density of the cable
    float tether_diameter = 0.1f;    // [m] radius of the cable
    float tether_init_length = 0.0f; // [m] length of the cable
    float gravity_const = 9.81f;     // [m/s^2] gravity constant
    float tether_mass = 0.0f;        // [kg] mass of the cable

    float tether_cur_length = 0.0f; // [m] current length of the cable
    float tether_grav_force = 0.0f;
    float tether_cur_ground_angle = 0.0f;
    float winch_force = 0.0f;

    // Condition variables
    bool is_node_alive = true; // always alive
    std::vector<uint8_t> sim_status = {0, 0, 0, 0, 0, 0, 0, 0};
    DisturbationMode disturb_mode_ = DisturbationMode::STRONG_SIDE; // to set manually

    float dist_gs_drone = 0.0f; // [m] distance between drone and ground station
    px4_msgs::msg::VehicleLocalPosition local_pos_latest;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr tether_force_pub_;

    // Publish functions
    void publishTetherForceDisturbations();

    // Susbcribers
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sim_status_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub;
    // Callback functions
    void simStatusSubCb(const std_msgs::msg::UInt8MultiArray msg);
    void vehicleLocalPositionSubCb(const px4_msgs::msg::VehicleLocalPosition msg);
  };

} // namespace tether_model
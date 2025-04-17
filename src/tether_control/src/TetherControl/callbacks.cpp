#include "tether_control/tether_control.hpp"

////////////////////////////////////// Callback functions //////////////////////////////////////

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

float winch_length = 0.0f;

namespace tether_control
{

  void TetherControl::vehicleAttitudeSubCb(const px4_msgs::msg::VehicleAttitude msg)
  {
    // storing local pos, to convert to lambda function if we don't do anything more with the sub
    Eigen::Quaterniond q = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg.q);
    Eigen::Quaterniond enu_q = px4_ros_com::frame_transforms::ned_to_enu_orientation(q);

    // Eigen::Quaterniond roll_correction(Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitY()));
    // Eigen::Quaterniond enu_q_fixed = roll_correction * enu_q;
    this->attitude_quat_latest = enu_q;

    RCLCPP_INFO_ONCE(this->get_logger(), "-------------- GOT ATTITUDE DATA --------------");
  }

  void TetherControl::vehicleLocalPositionSubCb(const px4_msgs::msg::VehicleLocalPosition msg)
  {
    // storing local pos, to convert to lambda function if we don't do anything more with the sub
    this->local_pos_latest = msg;

    float x = msg.y;
    float y = msg.x;
    float z = -msg.z;
    this->dist_gs_drone = std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
    this->tether_ground_cur_angle_theta = std::atan2(std::sqrt(std::pow(x, 2) + std::pow(y, 2)), z);
    this->tether_ground_cur_angle_phi = std::atan2(y, x);
    tether_model_.setTetherDroneDistance(this->dist_gs_drone);
    tether_model_.setWorldSpherAngles(this->tether_ground_cur_angle_theta, this->tether_ground_cur_angle_phi);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROT_FREQ, "Drone position: [%f, %f, %f]", msg.x,
                         msg.y, msg.z);
    if((msg.z <= -1.0) && (!this->is_init_pos))
      {
        RCLCPP_INFO_ONCE(this->get_logger(), "-------------- POSITION READY --------------");
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

  void TetherControl::winchJointStateSubCb(const sensor_msgs::msg::JointState msg)
  {
    // would get winch joint position and update tether length out
    float delta_angle_rad = this->winch_angle_latest - msg.position[0];
    float delta_tether_length = this->winch_diameter / 2.0 * delta_angle_rad; // r*delta_theta
    winch_length += delta_tether_length;
    this->winch_angle_latest = msg.position[0];

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROT_FREQ,
                         "Winch pos: %f, delta angle: %f, delta tether length: %f, winch length: %f", msg.position[0],
                         delta_angle_rad, delta_tether_length, winch_length);
  }

} // namespace tether_control
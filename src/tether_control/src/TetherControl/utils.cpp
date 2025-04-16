#include "tether_control/tether_control.hpp"

namespace tether_control
{

  void TetherControl::loadParams()
  {
    // Control
    this->log_mode = this->declare_parameter<bool>("log_mode", true);
    this->debug_mode = this->declare_parameter<bool>("debug_mode", true);
    this->tethered = this->declare_parameter<bool>("tethered", true);
    this->uav_type = this->declare_parameter<std::string>("uav_type", "MC");
    this->gravity = this->declare_parameter<float>("gravity", 9.81f);
    control_mode_s = this->declare_parameter<std::string>("control.control_mode", "ATTITUDE");
    this->attThrustKp = this->declare_parameter<float>("control.hoverThrust", 0.5f);
    this->attThrustKp = this->declare_parameter<float>("control.attThrustKp", 0.5f);
    this->attThrustKd = this->declare_parameter<float>("control.attThrustKd", 0.05f);
    this->attR = this->declare_parameter<float>("control.attR", 0.0f);
    this->attP = this->declare_parameter<float>("control.attP", 0.0f);
    this->attY = this->declare_parameter<float>("control.attY", 0.0f);
    // Model
    std::string disturb_mode_s = this->declare_parameter<std::string>("model.disturb_mode", "STRONG_SIDE");
    this->tether_init_length = this->declare_parameter<float>("model.tether_init_length", 85.7e9f);
    this->tether_diameter = this->declare_parameter<float>("model.tether_diameter", 0.005f);
    this->tether_density = this->declare_parameter<float>("model.tether_density", 970.0f);
    this->tether_yung_modulus = this->declare_parameter<float>("model.tether_yung_modulus", 970.0f);
    this->winch_diameter = this->declare_parameter<float>("model.winch_diameter", 970.0f);

    // Log for traceability
    RCLCPP_INFO(this->get_logger(), "------------------- PARAMETERS --------------------");
    RCLCPP_INFO(this->get_logger(), "------------------- CONTROL --------------------");
    RCLCPP_INFO(this->get_logger(), "log_mode: %i", this->log_mode);
    RCLCPP_INFO(this->get_logger(), "debug_mode: %i", this->debug_mode);
    RCLCPP_INFO(this->get_logger(), "tethered: %i", this->tethered);
    RCLCPP_INFO(this->get_logger(), "uav_type: %s", this->uav_type.c_str());
    RCLCPP_INFO(this->get_logger(), "gravity: %f", this->gravity);
    RCLCPP_INFO(this->get_logger(), "control_mode: %s", control_mode_s.c_str());
    RCLCPP_INFO(this->get_logger(), "attThrustKp: %f", this->attThrustKp);
    RCLCPP_INFO(this->get_logger(), "hoverThrust: %f", this->hoverThrust);
    RCLCPP_INFO(this->get_logger(), "attThrustKd: %f", this->attThrustKd);
    RCLCPP_INFO(this->get_logger(), "------------------- MODEL --------------------");
    RCLCPP_INFO(this->get_logger(), "disturb_mode: %s", disturb_mode_s.c_str());
    RCLCPP_INFO(this->get_logger(), "tether_init_length: %f", this->tether_init_length);
    RCLCPP_INFO(this->get_logger(), "tether_diameter: %f", this->tether_diameter);
    RCLCPP_INFO(this->get_logger(), "tether_density: %f", this->tether_density);
    RCLCPP_INFO(this->get_logger(), "winch_diameter: %f", this->winch_diameter);
    RCLCPP_INFO(this->get_logger(), "tether_yung_modulus: %f", this->tether_yung_modulus);
    RCLCPP_INFO(this->get_logger(), "---------------------------------------------------");

    convertControlMode(control_mode_s);
    convertDisturbMode(disturb_mode_s);
    setHoverThrust();

    // Setting model values
    tether_model_.setGravity(this->gravity);
    tether_model_.setTetherInitLength(this->tether_init_length);
    tether_model_.setTetherDiameter(this->tether_diameter);
    tether_model_.setTetherDensity(this->tether_density);
    tether_model_.setTetherYungModulus(this->tether_yung_modulus);
  }

  Eigen::Quaterniond TetherControl::rotateQuaternionFromToENU_NED(const Eigen::Quaterniond &quat_in)
  {
    // Transform from orientation represented in ROS format to PX4 format and back
    //  * Two steps conversion:
    //  * 1. aircraft to NED is converted to aircraft to ENU (NED_to_ENU conversion)
    //  * 2. aircraft to ENU is converted to baselink to ENU (baselink_to_aircraft conversion)
    // OR
    //  * 1. baselink to ENU is converted to baselink to NED (ENU_to_NED conversion)
    //  * 2. baselink to NED is converted to aircraft to NED (aircraft_to_baselink conversion)
    // NED_ENU_Q Static quaternion needed for rotating between ENU and NED frames
    Eigen::Vector3d euler_1(M_PI, 0.0, M_PI_2);
    Eigen::Quaterniond NED_ENU_Q(Eigen::AngleAxisd(euler_1.z(), Eigen::Vector3d::UnitZ())
                                 * Eigen::AngleAxisd(euler_1.y(), Eigen::Vector3d::UnitY())
                                 * Eigen::AngleAxisd(euler_1.x(), Eigen::Vector3d::UnitX()));

    // AIRCRAFT_BASELINK_Q Static quaternion needed for rotating between aircraft and base_link frames
    Eigen::Vector3d euler_2(M_PI, 0.0, 0.0);
    Eigen::Quaterniond AIRCRAFT_BASELINK_Q(Eigen::AngleAxisd(euler_2.z(), Eigen::Vector3d::UnitZ())
                                           * Eigen::AngleAxisd(euler_2.y(), Eigen::Vector3d::UnitY())
                                           * Eigen::AngleAxisd(euler_2.x(), Eigen::Vector3d::UnitX()));

    return (NED_ENU_Q * quat_in) * AIRCRAFT_BASELINK_Q;
  }

  Eigen::Vector3d TetherControl::quaternion_to_euler_deg(const Eigen::Quaterniond &q)
  {
    // Convert to Euler angles in roll-pitch-yaw order, then convert to degrees
    return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse() * (180.0 / M_PI);
  }

  double TetherControl::get_pitch_from_imu(const geometry_msgs::msg::Quaternion &quat)
  {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);  // Extract Euler angles
    return pitch * (180.0 / M_PI); // Convert to degrees
  }

  void TetherControl::convertControlMode(std::string control_mode_s)
  {
    if(control_mode_s == "POSITION")
      {
        this->control_mode = ControlMode::POSITION;
      }
    else if(control_mode_s == "ATTITUDE")
      {
        this->control_mode = ControlMode::ATTITUDE;
      }
    else if(control_mode_s == "DIRECT_ACTUATORS")
      {
        this->control_mode = ControlMode::DIRECT_ACTUATORS;
      }
    else if(control_mode_s == "TETHER_FORCE_REACTIONS")
      {
        this->control_mode = ControlMode::TETHER_FORCE_REACTIONS;
      }
    else if(control_mode_s == "CUSTOM")
      {
        this->control_mode = ControlMode::CUSTOM;
      }
    else if(control_mode_s == "NONE")
      {
        this->control_mode = ControlMode::NONE;
      }
    else
      {
        RCLCPP_ERROR(this->get_logger(), "Control mode not defined, setting to NONE");
        this->control_mode = ControlMode::NONE;
      }
  }

  void TetherControl::convertDisturbMode(std::string disturb_mode_s)
  {
    if(disturb_mode_s == "STRONG_SIDE")
      {
        this->disturb_mode = DisturbationMode::STRONG_SIDE;
      }
    else if(disturb_mode_s == "CIRCULAR")
      {
        this->disturb_mode = DisturbationMode::CIRCULAR;
      }
    else if(disturb_mode_s == "TET_GRAV_FIL_ANG")
      {
        this->disturb_mode = DisturbationMode::TET_GRAV_FIL_ANG;
      }
    else if(disturb_mode_s == "NONE")
      {
        this->disturb_mode = DisturbationMode::NONE;
      }
    else
      {
        RCLCPP_ERROR(this->get_logger(), "Disturbation mode not defined, setting to NONE");
        this->disturb_mode = DisturbationMode::NONE;
      }
  }

} // namespace tether_control
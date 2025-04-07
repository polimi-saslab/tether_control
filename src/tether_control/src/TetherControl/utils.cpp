#include "tether_control/tether_control.hpp"

namespace tether_control
{
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

  double TetherControl::get_pitch_from_imu(const geometry_msgs::msg::Quaternion &quat)
  {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);  // Extract Euler angles
    return pitch * (180.0 / M_PI); // Convert to degrees
  }

} // namespace tether_control
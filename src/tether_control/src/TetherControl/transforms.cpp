#include "tether_control/tether_control.hpp"

////////////////////////// Transform functions to help visualize the project in rviz2 ////////////////

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace tether_control
{
  void TetherControl::transformMapDrone()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "map";      // World frame (ENU)
    transform.child_frame_id = "base_link"; // Drone frame

    // Convert NED → ENU
    transform.transform.translation.x = this->local_pos_latest.y;  // East → X
    transform.transform.translation.y = this->local_pos_latest.x;  // North → Y
    transform.transform.translation.z = -this->local_pos_latest.z; // Down → -Z

    Eigen::Quaterniond q
      = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(this->attitude_latest.q);
    Eigen::Quaterniond enu_q = px4_ros_com::frame_transforms::ned_to_enu_orientation(q);

    // tf2::Quaternion q_ned(this->attitude_latest.q[1], this->attitude_latest.q[2], this->attitude_latest.q[3],
    //                       this->attitude_latest.q[0]);
    // tf2::Quaternion q_rotate;
    // q_rotate.setRPY(0.0, 0.0, -M_PI / 2.0);
    // tf2::Quaternion q_enu = q_rotate * q_ned;

    transform.transform.rotation.x = enu_q.x();
    transform.transform.rotation.y = enu_q.y();
    transform.transform.rotation.z = enu_q.z();
    transform.transform.rotation.w = enu_q.w();

    this->tf_broadcaster_->sendTransform(transform);
  }

} // namespace tether_control
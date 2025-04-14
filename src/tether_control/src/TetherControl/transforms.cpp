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

    transform.transform.rotation.x = this->attitude_quat_latest.x();
    transform.transform.rotation.y = this->attitude_quat_latest.y();
    transform.transform.rotation.z = this->attitude_quat_latest.z();
    transform.transform.rotation.w = this->attitude_quat_latest.w();

    this->tf_broadcaster_->sendTransform(transform);
  }

} // namespace tether_control
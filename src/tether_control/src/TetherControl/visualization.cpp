#include "tether_control/tether_control.hpp"

////////////////////////////////// Functions to help visualize the project in rviz2 //////////////////////////////////

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

    // Orientation: NED to ENU also needs conversion
    // Assuming msg->q is in NED, we must convert quaternion from NED to ENU
    // Simplest: convert using a static transform (90 deg rotation around Z)

    tf2::Quaternion q_ned(this->attitude_latest.q[0], this->attitude_latest.q[1], this->attitude_latest.q[2],
                          this->attitude_latest.q[3]);

    tf2::Quaternion q_rot;
    q_rot.setRPY(M_PI, 0, 0);

    tf2::Quaternion q_enu = q_rot * q_ned;
    q_enu.normalize();

    transform.transform.rotation.x = q_enu[0];
    transform.transform.rotation.y = q_enu[1];
    transform.transform.rotation.z = q_enu[2];
    transform.transform.rotation.w = q_enu[3];

    this->tf_broadcaster_->sendTransform(transform);
  }

} // namespace tether_control
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

    tf2::Quaternion q_ned(this->attitude_latest.q[1], this->attitude_latest.q[2], this->attitude_latest.q[3],
                          this->attitude_latest.q[0]);

    // Transform quaternion (NED to ENU)
    tf2::Quaternion q_rotate;
    q_rotate.setRPY(M_PI, 0.0, M_PI / 2.0); // 180° roll, 90° yaw
    tf2::Quaternion q_enu = q_rotate * q_ned;

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Drone ENU orientation: [%f, %f, %f, %f]",
    //                      q_enu[0], q_enu[1], q_enu[2], q_enu[3]);

    transform.transform.rotation.x = q_enu[1];
    transform.transform.rotation.y = q_enu[2];
    transform.transform.rotation.z = q_enu[3];
    transform.transform.rotation.w = q_enu[0];

    this->tf_broadcaster_->sendTransform(transform);
  }

} // namespace tether_control
// correctVehicle rel pos ENU: [0.853906, -2.430498, 1.205541], [dist_gs_drone, phi, theta]: [2.844259,
// -1.232937, 1.133104]
// correct :Vehicle rel pos ENU: [1.896133, 1.224420, 0.815073], [dist_gs_drone, phi, theta]: [2.399764,
// 0.573379, 1.224255] correct  Vehicle rel pos ENU: [2.026398, 0.559712, 0.829273], [dist_gs_drone, phi, theta]:
// [2.259925, 0.269491, 1.195071]
// correct: Vehicle rel pos ENU: [1.347961, 1.791242, 0.833599], [dist_gs_drone, phi, theta]: [2.391743,
// 0.925679, 1.214792]

// faux Vehicle rel pos ENU: [-0.549485, 0.560779, 1.103126], [dist_gs_drone, phi, theta]: [1.353992, 5.487616,
// 0.618548] faux  Vehicle rel pos ENU: [-0.478126, 0.641120, 1.069241], [dist_gs_drone, phi, theta]:
// [1.335259, 5.353175, 0.642209] faux Vehicle rel pos ENU: [-0.642168, -0.191742, 1.136107], [dist_gs_drone, phi,
// theta]: [1.319046, -5.993027, 0.532955]

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
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_ned).getRPY(roll, pitch, yaw);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROT_FREQ, "Drone RPY orientation: [%f, %f, %f]",
                         roll, pitch, yaw);

    // Transform quaternion (NED to ENU)
    tf2::Quaternion q_rotate;
    q_rotate.setRPY(M_PI, 0.0, M_PI / 2.0); // 180° roll, 90° yaw
    tf2::Quaternion q_enu = q_rotate * q_ned;

    transform.transform.rotation.x = q_enu[1];
    transform.transform.rotation.y = q_enu[2];
    transform.transform.rotation.z = q_enu[3];
    transform.transform.rotation.w = q_enu[0];

    this->tf_broadcaster_->sendTransform(transform);
  }

  void TetherControl::publish_marker()
  {
    // Create and fill the Marker message
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map"; // Reference frame (use "world" for global coordinates)
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "origin";                                // Namespace for the marker
    marker.id = 0;                                       // Unique ID for the marker
    marker.type = visualization_msgs::msg::Marker::CUBE; // Shape type (box)

    marker.action = visualization_msgs::msg::Marker::ADD; // Action (add the marker)

    // Set the position of the box at (0, 0, 0)
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    // Set orientation (no rotation)
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;

    // Set the scale (dimensions of the box)
    marker.scale.x = 0.05; // Length
    marker.scale.y = 0.05; // Width
    marker.scale.z = 0.01; // Height

    // Set the color of the box
    marker.color.r = 0.0f; // Red
    marker.color.g = 0.0f; // Green
    marker.color.b = 1.0f; // Blue
    marker.color.a = 1.0f; // Alpha (transparency)

    // Publish the marker
    origin_marker_pub_->publish(marker);
  }

} // namespace tether_control
#include "tether_model/tether_model.hpp"

using namespace std::chrono;

namespace tether_model
{
  TetherModel::TetherModel(const std::string &nodeName) : Node(nodeName)
  {
    RCLCPP_INFO(this->get_logger(), "################# TETHER MODEL NODE INITIALIZING #################");

    // Init parameters
    this->tether_density = this->declare_parameter<float>("tether_model.tether_density", 970.0f);
    this->tether_diameter = this->declare_parameter<float>("tether_model.tether_diameter", 0.005f);
    this->tether_init_length = this->declare_parameter<float>("tether_init_length", 1.0f);
    this->gravity_const = this->declare_parameter<float>("tether_model.gravity_const", 9.81f);
    std::string disturb_mode_s = this->declare_parameter<std::string>("disturb_mode", "STRONG_SIDE");

    RCLCPP_INFO(this->get_logger(), "------------------- PARAMETERS --------------------");
    RCLCPP_INFO(this->get_logger(), "tether_density: %f", this->tether_density);
    RCLCPP_INFO(this->get_logger(), "tether_diameter: %f", this->tether_diameter);
    RCLCPP_INFO(this->get_logger(), "tether_init_length: %f", this->tether_init_length);
    RCLCPP_INFO(this->get_logger(), "gravity_const: %f", this->gravity_const);
    RCLCPP_INFO(this->get_logger(), "disturb_mode: %s", disturb_mode_s.c_str());
    RCLCPP_INFO(this->get_logger(), "---------------------------------------------------");

    convertDisturbMode(disturb_mode_s);

    // Init publishers
    tether_force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/drone/tether_force", 10);

    // PX4 requires specific QoS, see
    // https://docs.px4.io/main/en/ros2/user_guide.html#compatibility-issues
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Init subscribers
    vehicle_local_position_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos,
      std::bind(&TetherModel::vehicleLocalPositionSubCb, this, std::placeholders::_1));
    vehicle_attitude_sub = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos, std::bind(&TetherModel::vehicleAttitudeSubCb, this, std::placeholders::_1));

    auto timer_alive_callback = [this]() -> void {
      if(this->is_node_alive)
        {
          RCLCPP_INFO_ONCE(this->get_logger(), "NODE ALIVE");
          return;
        }
      else
        {
          RCLCPP_INFO(this->get_logger(), "Tether not alive ...");
        }
    };

    auto timer_callback = [this]() -> void {
      if(!this->is_node_alive)
        {
          return;
        }
      else
        {
          if(this->is_init_pos)
            {
              publishTetherForceDisturbations();
            }
        }
    };

    timer_ = this->create_wall_timer(50ms, timer_callback);
    timer_alive_ = this->create_wall_timer(1000ms, timer_alive_callback);

    RCLCPP_INFO(this->get_logger(), "################# TETHER MODEL NODE INITIALIZED #################");
  }

  void TetherModel::vehicleAttitudeSubCb(const px4_msgs::msg::VehicleAttitude msg)
  {
    // storing local pos, to convert to lambda function if we don't do anything more with the sub
    this->attitude_latest = msg;

    // plotting once just for info
    RCLCPP_INFO_ONCE(this->get_logger(), "-------------- GOT ATTITUDE DATA --------------");
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Orientation: %f %f %f %f", msg.q[0], msg.q[1],
                         msg.q[2], msg.q[3]);
  }

  void TetherModel::publishTetherForceDisturbations()
  {
    geometry_msgs::msg::WrenchStamped msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = "map";

    if(this->disturb_mode == DisturbationMode::CIRCULAR)
      {
        static double theta = 0.0; // angle for circular motion
        double radius = 1.5;       // radius of the circle (i.e., force magnitude in XY)
        double d_theta = 0.01;     // increment angle each call (controls speed)

        msg.wrench.force.x = radius * std::cos(theta);
        msg.wrench.force.y = radius * std::sin(theta);
        msg.wrench.force.z = -5.0; // Constant downward force
        msg.wrench.torque.x = 0.0;
        msg.wrench.torque.y = 0.0;
        msg.wrench.torque.z = 0.0;

        theta += d_theta;
        if(theta > 2 * M_PI)
          theta -= 2 * M_PI;
      }
    else if(this->disturb_mode == DisturbationMode::STRONG_SIDE)
      {
        msg.wrench.force.x = -2.5;
        msg.wrench.force.y = -2.5;
        msg.wrench.force.z = -2.5;
        msg.wrench.torque.x = 0.0;
        msg.wrench.torque.y = 0.0;
        msg.wrench.torque.z = 0.0;
      }
    else if(this->disturb_mode == DisturbationMode::TET_GRAV_FIL_ANG)
      {
        // Specific model for tether force
        this->tether_cur_length = this->dist_gs_drone; // easy way atm

        float tether_mass
          = this->tether_density * M_PI * std::pow(this->tether_diameter / 2, 2) * this->tether_cur_length; // [kg]
        float tether_ang_due_to_grav
          = 1 / 2
            * (tether_mass * this->gravity_const * std::sin(this->tether_ground_cur_angle_theta)
               / this->winch_force); // == 1/2(M_t*g*cos(beta)/T), from eq (39) of The Influence of
        // Tether Sag on Airborne Wind Energy Generation by F. Trevisi
        // need to rotate vector from world to ENU, since publishing tether_force
        // in ENU frame
        // this->tether_drone_cur_angle = this->tether_ground_cur_angle_theta;

        float tether_grav_force
          = tether_mass * this->gravity_const; // [N] force on drone due to gravity of tether weight

        // assuming spherical coordinates for ENU frame:
        msg.wrench.force.x = (tether_grav_force + this->winch_force) * std::sin(this->tether_ground_cur_angle_theta)
                             * std::cos(this->tether_ground_cur_angle_phi);
        msg.wrench.force.y = (tether_grav_force + this->winch_force)
                             * std::sin(tether_ang_due_to_grav + this->tether_ground_cur_angle_theta)
                             * std::sin(this->tether_ground_cur_angle_phi);
        msg.wrench.force.z
          = (tether_grav_force + this->winch_force)
            * std::cos(tether_ang_due_to_grav
                       + this->tether_ground_cur_angle_theta); // should be pi - theta, then z=-z but same
        RCLCPP_DEBUG(this->get_logger(),
                     "tether_cur_length: %f, tether_mass: %f, tether_ang_due_to_grav: %f, "
                     "tether_ground_cur_angle_theta: %f, tether_ground_cur_angle_phi: %f",
                     this->tether_cur_length, tether_mass, tether_ang_due_to_grav, this->tether_ground_cur_angle_theta,
                     this->tether_ground_cur_angle_phi);
      }
    else
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), LOG_THROTTLE, "Disturbation mode not defined");
      }

    // sanity check
    if(std::isnan(msg.wrench.force.x) || std::isnan(msg.wrench.force.y) || std::isnan(msg.wrench.force.z))
      {
        RCLCPP_WARN(this->get_logger(), "Nan values in msg force, sanitizing to 0");
        msg.wrench.force.x = 0.0;
        msg.wrench.force.y = 0.0;
        msg.wrench.force.z = 0.0;
        msg.wrench.torque.x = 0.0;
        msg.wrench.torque.y = 0.0;
        msg.wrench.torque.z = 0.0;
      }

    // Eigen::Quaterniond q_world_drone(this->attitude_latest.q[0], // w
    //                                  this->attitude_latest.q[1], // x
    //                                  this->attitude_latest.q[2], // y
    //                                  this->attitude_latest.q[3]  // z
    // );

    // Eigen::Quaterniond q_ned(this->attitude_latest.q[0], // w
    //                          this->attitude_latest.q[1], // x
    //                          this->attitude_latest.q[2], // y
    //                          this->attitude_latest.q[3]  // z
    // );

    // // Create the fixed rotation to convert NED to ENU: RPY(180°, 0°, 90°)
    // Eigen::AngleAxisd roll180(M_PI, Eigen::Vector3d::UnitX());
    // Eigen::AngleAxisd pitch0(0.0, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd yaw90(M_PI / 2.0, Eigen::Vector3d::UnitZ());

    // // Combine rotations: R = Yaw * Pitch * Roll (intrinsic rotations)
    // Eigen::Quaterniond q_rotate = yaw90 * roll180;

    // // Convert PX4 NED to ENU world orientation
    // Eigen::Quaterniond q_enu = q_rotate * q_ned;

    // // Wrench force from ROS2 (ENU) — already converted from NED manually?
    tf2::Vector3 F_world(-msg.wrench.force.x, -msg.wrench.force.y, msg.wrench.force.z);

    tf2::Quaternion q_ned(this->attitude_latest.q[1], this->attitude_latest.q[2], this->attitude_latest.q[3],
                          this->attitude_latest.q[0]);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_ned).getRPY(roll, pitch, yaw);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), LOG_THROTTLE, "Drone RPY orientation: [%f, %f, %f]",
                         roll, pitch, yaw);

    // Transform quaternion (NED to ENU)
    tf2::Quaternion q_rotate;
    q_rotate.setRPY(M_PI, 0.0, M_PI / 2.0); // 180° roll, 90° yaw
    tf2::Quaternion q_enu = q_rotate * q_ned;

    tf2::Matrix3x3 rot_matrix(q_enu);
    // tf2::Vector3 rotated = tf2::quatRotate(q_enu, F_world);

    // // Rotate force into drone's body frame
    tf2::Vector3 F_body = F_world * rot_matrix;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Drone Attitude orientation: [%f, %f, %f, %f]",
                         this->attitude_latest.q[0], this->attitude_latest.q[1], this->attitude_latest.q[2],
                         this->attitude_latest.q[3]);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Drone ENU orientation: [%f, %f, %f, %f]",
                         q_ned[0], q_ned[1], q_ned[2], q_ned[3]);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                         "Drone force before: [%f, %f, %f] and ENU Force: [%f, %f, %f]", -msg.wrench.force.x,
                         -msg.wrench.force.y, msg.wrench.force.z, F_body[0], F_body[1], F_body[2]);

    // inverse, to match drone's view
    // msg.wrench.force.x = F_body[0];
    // msg.wrench.force.y = F_body[1];
    // msg.wrench.force.z = F_body[2];
    // msg.wrench.force.x = -msg.wrench.force.x; // felt by drone
    // msg.wrench.force.y = -msg.wrench.force.y; // felt by drone
    // msg.wrench.force.z = msg.wrench.force.z;  // felt by drone

    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 50, "Publishing tether forces [%f, %f, %f]",
                         msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
    this->tether_force_pub_->publish(msg);
  }

  void TetherModel::vehicleLocalPositionSubCb(const px4_msgs::msg::VehicleLocalPosition msg)
  {
    this->local_pos_latest = msg; // NED, whereas ROS is ENU -> convert
    // [m] distance between drone and ground station, NED frame but doesn't matter here
    this->dist_gs_drone = std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2) + std::pow(msg.z, 2));

    // local NED_TO_ENU conversion (PX4 to ROS)
    float x = msg.y;
    float y = msg.x;
    float z = -msg.z;

    // compute cartesian -> spherical conversion for the angles
    if(z > 0)
      {
        this->tether_ground_cur_angle_theta = std::atan2(std::sqrt(std::pow(x, 2) + std::pow(y, 2)), z);
      }
    else if(z < 0)
      {
        this->tether_ground_cur_angle_theta = M_PI + std::atan2(std::sqrt(std::pow(x, 2) + std::pow(y, 2)), z);
      }
    else if((z = 0.0) && (std::sqrt(std::pow(x, 2) + std::pow(y, 2)) != 0))
      {
        this->tether_ground_cur_angle_theta = M_PI / 2.0;
      }
    else if((x == 0.0f) && (y == 0.0f) && (z == 0.0f))
      {
        this->tether_ground_cur_angle_theta = 0.0; // undefined
      }
    // sgn(y) * std::acos(x / std::sqrt(std::pow(x, 2) + std::pow(y, 2)));
    //  std::atan2(std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2)), msg.z);
    if(x > 0)
      {
        this->tether_ground_cur_angle_phi = std::atan2(y, x);
      }
    else if((x < 0) && (y >= 0.0))
      {
        this->tether_ground_cur_angle_phi = M_PI + std::atan2(y, x);
      }
    else if((x < 0) && (y < 0.0))
      {
        this->tether_ground_cur_angle_phi = -M_PI + std::atan2(y, x);
      }
    else if((x == 0) && (y > 0.0))
      {
        this->tether_ground_cur_angle_phi = M_PI / 2.0;
      }
    else if((x == 0) && (y < 0.0))
      {
        this->tether_ground_cur_angle_phi = -M_PI / 2.0;
      }
    else if((x == 0.0) && (y == 0.0)) // undefined
      {
        this->tether_ground_cur_angle_phi = 0.0;
      }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                         "Vehicle rel pos ENU: [%f, %f, %f], [dist_gs_drone, phi, theta]: [%f, %f, %f]", x, y, z,
                         this->dist_gs_drone, this->tether_ground_cur_angle_phi, this->tether_ground_cur_angle_theta);

    if((msg.z <= -1.0) && (!this->is_init_pos))
      {
        RCLCPP_INFO_ONCE(this->get_logger(), "-------------- POSITION READY --------------");
        this->is_init_pos = true;
      }
  }

  void TetherModel::computeTetherForceVec()
  {
    return;
    // float phi = 0.0f; // [rad] tether angle in xy plane
  }

  void TetherModel::convertDisturbMode(std::string disturb_mode_s)
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

} // namespace tether_model

int main(int argc, char *argv[])
{
  std::cout << "Starting tether model node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  const char *node_user = std::getenv("USER");
  auto node = std::make_shared<tether_model::TetherModel>(node_user ? std::string(node_user) + "_tether_model_node"
                                                                    : "tether_model_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
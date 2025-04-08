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
    std::string disturb_mode_s = this->declare_parameter<std::string>("disturb_mode", "NONE");

    RCLCPP_INFO(this->get_logger(), "------------------- PARAMETERS --------------------");
    RCLCPP_INFO(this->get_logger(), "tether_density: %f", this->tether_density);
    RCLCPP_INFO(this->get_logger(), "tether_diameter: %f", this->tether_diameter);
    RCLCPP_INFO(this->get_logger(), "tether_init_length: %f", this->tether_init_length);
    RCLCPP_INFO(this->get_logger(), "gravity_const: %f", this->gravity_const);
    RCLCPP_INFO(this->get_logger(), "disturb_mode: %s", disturb_mode_s.c_str());
    RCLCPP_INFO(this->get_logger(), "---------------------------------------------------");

    convertModeStringToEnum(disturb_mode_s);

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
    sim_status_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/tether_control/sim_status", qos, std::bind(&TetherModel::simStatusSubCb, this, std::placeholders::_1));

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
          if(sim_status[0] == 0) // msg that tether_control would send when rdy, atm is ignored
            {
              RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for simulation mode to be set");
              publishTetherForceDisturbations();
            }
          else
            {
              RCLCPP_INFO_ONCE(this->get_logger(), "ALIVE");
              publishTetherForceDisturbations();
            }
        }
    };

    timer_ = this->create_wall_timer(50ms, timer_callback);
    timer_alive_ = this->create_wall_timer(1000ms, timer_alive_callback);

    RCLCPP_INFO(this->get_logger(), "################# TETHER MODEL NODE INITIALIZED #################");
  }

  void TetherModel::simStatusSubCb(const std_msgs::msg::UInt8MultiArray msg)
  {
    this->sim_status = msg.data;
    RCLCPP_INFO_ONCE(this->get_logger(), "Received Sim Status %d", this->sim_status[0]);
  }

  void TetherModel::publishTetherForceDisturbations()
  {
    geometry_msgs::msg::WrenchStamped msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    // // initializing to avoid nan errors with gz
    // msg.wrench.force.x = 0.0;
    // msg.wrench.force.y = 0.0;
    // msg.wrench.force.z = 0.0;
    // msg.wrench.torque.x = 0.0;
    // msg.wrench.torque.y = 0.0;
    // msg.wrench.torque.z = 0.0;

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
        msg.wrench.force.x = -2.0;
        msg.wrench.force.y = -2.0;
        msg.wrench.force.z = -2.0;
        msg.wrench.torque.x = 0.0;
        msg.wrench.torque.y = 0.0;
        msg.wrench.torque.z = 0.0;
      }
    else if(this->disturb_mode == DisturbationMode::TET_GRAV_FIL_ANG)
      {
        // Specific model for tether force
        float tether_mass
          = this->tether_density * M_PI * std::pow(this->tether_diameter / 2, 2) * this->tether_cur_length; // [kg]
        this->tether_ground_cur_angle_phi
          = 1 / 2
            * (tether_mass * this->gravity_const * cos(this->tether_ground_cur_angle_phi)
               / this->winch_force); // == 1/2(M_t*g*cos(beta)/T), from eq (39) of The Influence of
                                     // Tether Sag on Airborne Wind Energy Generation by F. Trevisi
                                     // need to rotate vector from world to ENU, since publishing tether_force
                                     // in ENU frame
        float tether_grav_force
          = tether_mass * this->gravity_const; // [N] force on drone due to gravity of tether weight

        // assuming spherical coordinates for ENU frame:
        msg.wrench.force.x = (tether_grav_force + this->winch_force) * cos(this->tether_ground_cur_angle_phi)
                             * cos(this->tether_ground_cur_angle_theta);
        msg.wrench.force.y = (tether_grav_force + this->winch_force) * cos(this->tether_ground_cur_angle_phi)
                             * sin(this->tether_ground_cur_angle_theta);
        msg.wrench.force.z = (tether_grav_force + this->winch_force) * sin(this->tether_ground_cur_angle_phi);

        RCLCPP_DEBUG(this->get_logger(), "Computed tether gravity force: %f, winch force: %f", tether_grav_force,
                     this->winch_force);
      }
    else
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), LOG_THROTTLE, "Disturbation mode not defined");
      }

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

    RCLCPP_INFO(get_logger(), "Publishing tether forces [%f, %f, %f]", msg.wrench.force.x, msg.wrench.force.y,
                msg.wrench.force.z);
    this->tether_force_pub_->publish(msg);
  }

  void TetherModel::vehicleLocalPositionSubCb(const px4_msgs::msg::VehicleLocalPosition msg)
  {
    this->local_pos_latest = msg;
    RCLCPP_INFO_ONCE(this->get_logger(), "Received vehicle local position");
    // [m] distance between drone and ground station, NED frame but doesn't matter here
    this->dist_gs_drone = std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2) + std::pow(msg.z, 2));

    // compute cartesian -> spherical conversion for the angles
    this->tether_ground_cur_angle_theta = std::atan2(std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2)), msg.z);
    this->tether_ground_cur_angle_phi = std::atan2(msg.y, msg.x);
    RCLCPP_DEBUG(this->get_logger(),
                 "Vehicle rel pos: [%f, %f, %f], distance & angle between drone and ground station: [%f, %f, %f]",
                 msg.x, msg.y, msg.z, this->dist_gs_drone, this->tether_ground_cur_angle_phi,
                 this->tether_ground_cur_angle_theta);
  }

  void TetherModel::computeTetherForceVec()
  {
    return;
    // float phi = 0.0f; // [rad] tether angle in xy plane
  }

  void TetherModel::convertModeStringToEnum(std::string disturb_mode_s)
  {
    if(disturb_mode_s == "NONE")
      {
        this->disturb_mode = DisturbationMode::NONE;
      }
    else if(disturb_mode_s == "STRONG_SIDE")
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
    else
      {
        RCLCPP_ERROR(this->get_logger(), "Disturbation mode not defined");
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
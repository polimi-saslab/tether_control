#include "tether_model/tether_model.hpp"

using namespace std::chrono;

namespace tether_model
{
  TetherModel::TetherModel(const std::string &nodeName) : Node(nodeName)
  {
    RCLCPP_INFO(this->get_logger(), "################# TETHER MODEL NODE INITIALIZING #################");

    // Init parameters
    this->tether_density = this->declare_parameter<float>("tether_model.tether_density", 10.0f);
    this->tether_diameter = this->declare_parameter<float>("tether_model.tether_diameter", 0.005f);
    this->tether_init_length = this->declare_parameter<float>("tether_init_length", 1.0f);
    this->gravity_const = this->declare_parameter<float>("tether_model.gravity_const", 9.81f);

    RCLCPP_INFO(this->get_logger(), "------------------- PARAMETERS --------------------");
    RCLCPP_INFO(this->get_logger(), "tether_density: %f", this->tether_density);
    RCLCPP_INFO(this->get_logger(), "tether_diameter: %f", this->tether_diameter);
    RCLCPP_INFO(this->get_logger(), "tether_init_length: %f", this->tether_init_length);
    RCLCPP_INFO(this->get_logger(), "gravity_const: %f", this->gravity_const);

    // RCLCPP_INFO(this->get_logger(), "disturbation_mode: %s", this->disturb_mode_);
    RCLCPP_INFO(this->get_logger(), "---------------------------------------------------");

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
          if(sim_status[0] == 0)
            {
              RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for simulation mode to be set");
              publishTetherForceDisturbations();
            }
          else
            {
              RCLCPP_INFO(this->get_logger(), "ALIVE");
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
    if(this->disturb_mode_ == DisturbationMode::CIRCULAR)
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
    else if(this->disturb_mode_ == DisturbationMode::STRONG_SIDE)
      {
        msg.wrench.force.x = -2.0;
        msg.wrench.force.y = -2.0;
        msg.wrench.force.z = -2.0;
        msg.wrench.torque.x = 0.0;
        msg.wrench.torque.y = 0.0;
        msg.wrench.torque.z = 0.0;
      }
    else if(this->disturb_mode_ == DisturbationMode::CUSTOM)
      {
        this->tether_mass
          = this->tether_density * M_PI * std::pow(this->tether_diameter / 2, 2) * this->tether_cur_length; // [kg]
        this->tether_grav_force
          = this->tether_mass * this->gravity_const; // [N] force on drone due to gravity of tether weight
        // tether_tension_force = this->winch_force;
        this->tether_cur_ground_angle
          = 1 / 2
            * (this->tether_mass * this->gravity_const * cos(this->tether_cur_ground_angle)
               / this->winch_force); // == 1/2(M_t*g*cos(beta)/T), from eq (39) of The Influence of
                                     // Tether Sag on Airborne Wind Energy Generation by F. Trevisi
        RCLCPP_INFO(this->get_logger(), "Computed tether mass: %f, tether grav force: %f, tether drone angle: %f",
                    this->tether_mass, this->tether_grav_force, this->tether_cur_ground_angle);
      }
    else
      {
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), LOG_THROTTLE, "Disturbation mode not defined");
      }

    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), LOG_THROTTLE, "Publishing tether forces [%f, %f, %f]",
                         msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
    this->tether_force_pub_->publish(msg);
  }

  void TetherModel::vehicleLocalPositionSubCb(const px4_msgs::msg::VehicleLocalPosition msg)
  {
    this->local_pos_latest = msg;
    RCLCPP_INFO_ONCE(this->get_logger(), "Received vehicle local position");
    this->dist_gs_drone = std::sqrt(
      std::pow(msg.x, 2) + std::pow(msg.y, 2)
      + std::pow(msg.z, 2)); // [m] distance between drone and ground station, NED frame but doesn't matter here
    RCLCPP_INFO(this->get_logger(), "Distance between drone and ground station: %f", this->dist_gs_drone);
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
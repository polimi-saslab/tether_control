#ifndef TETHER_CONTROL_TETHER_MODEL_H
#define TETHER_CONTROL_TETHER_MODEL_H

#include <eigen3/Eigen/Eigen>

class tether_model
{
public:
  tether_model();
  void computeTetherVecForce(double *tether_force_vec, double *delta_theta);

  // Setters
  void setOdometry(const Eigen::Vector3d &position_W, const Eigen::Quaterniond &orientation_B_W,
                   const Eigen::Vector3d &velocity_B, const Eigen::Vector3d &angular_velocity_B)
  {
    R_B_W_ = orientation_B_W.toRotationMatrix(); // rotation matrix from body to world frame
    position_W_ = position_W;
    velocity_W_ = R_B_W_ * velocity_B;
    angular_velocity_B_ = angular_velocity_B;
  }

  void setGravity(double gravity) { gravity_ = gravity; }
  void setTetherDiameter(double diameter) { diameter_ = diameter; }
  void setTetherDensity(double density) { density_ = density; }
  void setTetherYungModulus(double young_modulus) { yung_modulus_ = young_modulus; }
  void setTetherInitLength(double init_length) { init_length_ = init_length; }
  void setTetherCurLength(double cur_length) { cur_length_ = cur_length; }
  void setTetherDroneDistance(double drone_gs_dist) { drone_gs_dist_ = drone_gs_dist; }
  void setWinchReactionForce(double winch_force) { winch_force_ = winch_force; }
  void setWorldSpherAngles(double theta, double phi)
  {
    polar_theta = theta;
    azim_phi_ = phi;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  double gravity_;
  double density_;
  double yung_modulus_;
  double diameter_;
  double init_length_;

  // Current states of tether cable
  double cur_length_ = 1.0;
  double drone_gs_dist_;
  double winch_force_ = 1.0;
  double polar_theta = 0.0; // [rad] spher angle between xy plane and z-axis
  double azim_phi_ = 0.0;   // [rad] spher angle between x & y axes

  // Current states of drone
  Eigen::Vector3d position_W_;
  Eigen::Vector3d velocity_W_;
  Eigen::Matrix3d R_B_W_;
  Eigen::Vector3d angular_velocity_B_;
};

#endif // TETHER_CONTROL_TETHER_MODEL_H
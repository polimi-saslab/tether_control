#include "../include/tether_control/model.h"
#include <cmath>
#include <fstream>
#include <iomanip> // For setting the precision
#include <iostream>

#include <stdint.h>

tether_model::tether_model() {}

void tether_model::computeTetherVecForce(double *resulting_force, double *delta_theta)
{
  assert(resulting_force);

  cur_length_ = drone_gs_dist_; // TO REPLACE

  double cable_section_ = M_PI * std::pow(diameter_ / 2, 2);
  // k = EA/L
  double spring_constant_ = yung_modulus_ * cable_section_ / cur_length_; // [N/m]

  // Force on drone due to gravity of tether weight, F_g = g*rho*A*L
  double tether_grav_force = density_ * cable_section_ * cur_length_ * gravity_;

  // delta angle due to tether weight == 1/2(M_t*g*cos(beta)/T),
  // from eq (39) of The Influence of Tether Sag on Airborne Wind Energy Generation by F. Trevisi
  double tether_ang_due_to_grav = 1 / 2 * (tether_grav_force * std::sin(polar_theta) / winch_force_);

  // spring force, we consider only the reaction one (min is 0)
  double tether_spring_force = std::max(0.0, spring_constant_ / init_length_ * (drone_gs_dist_ - init_length_));

  // in ENU frame
  *resulting_force = tether_spring_force + tether_grav_force + winch_force_;
  *delta_theta = tether_ang_due_to_grav;

  std::cout << "------------------------ Model 1 ------------------------ " << std::endl;
  std::cout << "Tether Tensile Force: " << tether_spring_force << " N" << std::endl;
  std::cout << "Tether Gravitational Force: " << tether_grav_force << " N" << std::endl;
  std::cout << "Winch Force: " << winch_force_ << " N" << std::endl;
  std::cout << "Resulting Force: " << *resulting_force << " N" << std::endl;
  std::cout << "Delta theta: " << *delta_theta << " [rad]" << std::endl;
}
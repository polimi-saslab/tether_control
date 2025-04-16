#include "../include/tether_control/tether_model.h"
#include <cmath>
#include <fstream>
#include <iomanip> // For setting the precision
#include <iostream>

#include <stdint.h>

tether_model::tether_model() {}

void tether_model::computeTetherForce(float *tether_force_vec)
{
  assert(tether_force_vec);

  float cable_section_ = M_PI * std::pow(diameter_ / 2, 2);
  double spring_constant_ = yung_modulus_ * cable_section_ / cur_length_; // [N/m]
  // Specific model for tether force                                                           // easy way atm
  float tether_mass = density_ * cable_section_ * cur_length_; // [kg]
  float tether_grav_force = tether_mass * gravity_;            // [N] force on drone due to gravity of tether weight
  float tether_ang_due_to_grav = 1 / 2
                                 * (tether_grav_force * std::sin(tether_ground_cur_angle_theta)
                                    / winch_force_); // == 1/2(M_t*g*cos(beta)/T), from eq (39) of The Influence of
  float tether_tensile_force = std::max(0.0, spring_constant_ / init_length_ * (drone_gs_dist_ - init_length_));
  // k*(r-l0)/l0, where k = EA/L = 85.7e9 * M_PI*(0.01/2)² / tether_cur_length
  // Tether Sag on Airborne Wind Energy Generation by F. Trevisi
  // need to rotate vector from world to ENU, since publishing tether_force
  // in ENU frame

  float resulting_force = tether_tensile_force + tether_grav_force + winch_force_;
}
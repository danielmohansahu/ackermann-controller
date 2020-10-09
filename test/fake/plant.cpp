/* @file plant.cpp
 */

#include "plant.h"

namespace fake
{

void Plant::setState(double speed, double heading)
{
  speed_ = speed;
  heading_ = heading;
}

void Plant::getState(double& speed, double& heading)
{
  speed = speed_;
  heading = heading_;
}

void Plant::applyCommand(double w_left, double w_right, double dt)
{
  // @TODO add noise

  // check for no motion
  if (w_left == 0.0 && w_right == 0)
    return;

  // simulate the application of the given commanded wheel velocities
  double w_o, w_i;
  if (std::abs(w_left) >= std::abs(w_right))
  {
    w_o = w_left;
    w_i = w_right;
  }
  else
  {
    w_i = w_left;
    w_o = w_right;
  }
  
  // limit based on max_steering_angle; model this as wheel slip on faster (outer) wheel
  // @TODO Daniel Sahu include derivation?
  // @TODO check for unreal results and if roots are both valid (??)
  double alpha = std::pow(w_i / w_o, 2);
  double r_1 = 0.5 * opts_.wheel_separation * (alpha + 1)/(alpha - 1);
  double r_2 = 0.5 * std::sqrt(std::pow(opts_.wheel_separation, 2) * ((alpha + 1)/(alpha - 1) - 1) - 4 * std::pow(opts_.wheel_separation, 2));
  double r = r_1 + r_2;

  // check if we've violated the maximum steering angle
  //  note that we only need to check the inner wheel steering angle
  double steering_angle_i = std::atan(opts_.wheel_base / (r - 0.5 * opts_.wheel_separation));
  if (std::abs(steering_angle_i) > opts_.max_steering_angle * M_PI/180.)
  {
    // @TODO actually do something here to limit the results
    std::cerr << "Violated max steering angle: " << steering_angle_i << " vs " << opts_.max_steering_angle * M_PI/180. << std::endl;
  }

  // calculate speed
  speed_ = M_PI * opts_.wheel_radius * (w_o + w_i);
  
  // calculate resulting heading by integrating across dt
  heading_ += dt * speed_ / r;
}

} // namespace fake
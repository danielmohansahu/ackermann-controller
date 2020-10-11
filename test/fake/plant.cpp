/* @file plant.cpp
 * @copyright [2020]
 */

#include "plant.h"

namespace fake {

void Plant::setState(double speed, double heading) {
  speed_ = speed;
  heading_ = heading;
}

void Plant::getState(double& speed, double& heading) {
  speed = speed_;
  heading = heading_;
}

void Plant::applyCommand(double w_left, double w_right, double dt) {
  // check for no motion (which resets speed)
  double eps = std::numeric_limits<double>::epsilon();
  if (std::abs(w_left) < eps && std::abs(w_right) < eps) {
    speed_ = 0.0;
    return;
  }

  // simulate the application of the given commanded wheel velocities
  double w_o, w_i;
  if (std::abs(w_left) >= std::abs(w_right)) {
    w_o = w_left;
    w_i = w_right;
  } else {
    w_i = w_left;
    w_o = w_right;
  }

  // limit based on max_steering_angle; model this as wheel
  // slip on faster (outer) wheel
  // @TODO check for unreal results and if roots are both valid (??)
  double alpha = (w_i * w_i + w_o * w_o) / (w_o * w_o - w_i * w_i);
  double r_1 = 0.5 * opts_.wheel_separation * alpha;
  double r_2 = 0.5 * std::sqrt(std::pow(opts_.wheel_separation, 2.0)
                                        * (alpha * alpha - 1)
                                        - 4 * std::pow(opts_.wheel_base, 2.0));
  double r = r_1 + r_2;

  // check if we've violated the maximum steering angle
  //  note that we only need to check the inner wheel steering angle
  double steering_angle_i = std::atan(opts_.wheel_base
                            / (r - 0.5 * opts_.wheel_separation));
  if (std::abs(steering_angle_i) > opts_.max_steering_angle * M_PI/180.)
    // @TODO Daniel Sahu actually do something here to limit the results
    std::cerr << "Violated max steering angle: "
              << steering_angle_i << " vs "
              << opts_.max_steering_angle * M_PI / 180. << std::endl;

  // calculate speed (instantaneously achieved) with some noise added in
  speed_ = M_PI * opts_.wheel_radius * (w_o + w_i) + dist_(generator_);

  // calculate resulting heading by integrating across dt (and add noise)
  heading_ += dt * speed_ / r + dist_(generator_);
}

} // namespace fake

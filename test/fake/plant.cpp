/* @file plant.cpp
 * @copyright [2020]
 */

#include "plant.h"

namespace fake {

void Plant::reset() {
  speed_ = 0.0;
  heading_ = 0.0;
}

void Plant::setState(const double speed, const double heading) {
  speed_ = speed;
  heading_ = heading;
}

void Plant::getState(double& speed, double& heading) const {
  speed = speed_;
  heading = heading_;
}

void Plant::command(const double throttle, const double steering, const double dt) {

  // sanity check inputs
  double steering_capped = steering;
  if (std::abs(steering) > opts_.max_steering_angle) {
    std::cout << "Given a commanded steering angle beyond our limit ("
              << steering << " vs. " << opts_.max_steering_angle << ")"
              << std::endl;
    // if we're debugging, this is a failure
    assert(false);
    steering_capped = ((steering > 0) - (steering < 0)) * opts_.max_steering_angle;
  }

  // convert to radians
  steering_capped *= M_PI / 180.0;

  // throttle translates directly to speed, since we have no other system knowledge
  speed_ = throttle;

  // the global heading is affected by our speed, steering angle, and wheel base
  heading_ += dt * (speed_ / opts_.wheel_base) * std::tan(steering_capped);

  // add in some noise for good measure
  speed_ += dist_(generator_);
  heading_ += dist_(generator_);
}

} // namespace fake

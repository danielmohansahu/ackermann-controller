/* @file Limits.cpp
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Sentosh Kesani
 *
 * @copyright [2020]
 */

// @TODO Currently STUB implementation; needs to be filled

#include <Limits.hpp>

namespace ackermann {

Limits::Limits(const std::shared_ptr<const Params>& params)
  : params_(params) {
}

void Limits::limit(const double current_throttle,
                   const double current_steering,
                   const double current_steering_vel,
                   double& desired_throttle,
                   double& desired_steering,
                   double& desired_steering_vel,
                   double dt) const {

    // first: limit current_throttle to [0,1]
    if (desired_throttle > 1) {desired_throttle = 1;}
    if (desired_throttle < 0) {desired_throttle = 0;}

    double throttle_vel = (desired_throttle - current_throttle) * dt;
    // max rate of change of throttle is "acceleration_max"
    if (throttle_vel > params_->acceleration_max)
      {desired_throttle = current_throttle + params_->acceleration_max;}
    if (throttle_vel < params_->acceleration_min)
      {desired_throttle = current_throttle + params_->acceleration_min;}


    double heading_vel;
    double heading_accel;
    // limit heading my max angle
    if ((desired_steering - current_steering) > params_->max_steering_angle) {
      desired_steering = params_->max_steering_angle;
    }
    if ((desired_steering - current_steering) < -params_->max_steering_angle) {
      desired_steering = -params_->max_steering_angle;
    }

    // limit heading by max angle rate of change
    heading_vel = (desired_steering - current_steering) * dt;
    if (heading_vel > params_->angular_velocity_max) {
      heading_vel = params_->angular_velocity_max;
      desired_steering = current_steering + heading_vel/dt;
    }
    if (heading_vel < params_->angular_velocity_min) {
      heading_vel = params_->angular_velocity_min;
      desired_steering = current_steering + heading_vel/dt;
    }

    // limit heading by max angle rate of change rate of change
    heading_accel = (heading_vel - current_steering_vel) * dt;
    if (heading_accel > params_->angular_acceleration_max) {
      heading_accel = params_->angular_acceleration_max;
      desired_steering_vel = current_steering_vel + heading_accel/dt;
      desired_steering = current_steering + desired_steering_vel/dt;

    }
    if (heading_accel < params_->angular_acceleration_min) {
      heading_accel = params_->angular_acceleration_min;
      desired_steering_vel = current_steering_vel + heading_accel/dt;
      desired_steering = current_steering + desired_steering_vel/dt;
    }

}

} // namespace ackermann

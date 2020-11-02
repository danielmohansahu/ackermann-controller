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

double Limits::throttleToSpeed(const double throttle) const {
  double speed_calc;
  if (throttle <= 0) {
    speed_calc = 0;
  }  else {
    speed_calc = throttle * params_->velocity_max;
  }
  return speed_calc;
}

double Limits::speedToThrottle(const double speed) const {
  double throttle_calc;
  if (speed <= 0) {
    throttle_calc = 0;
  }  else {
    throttle_calc = speed / params_->velocity_max;
  }
  return throttle_calc;
}

void Limits::limit(const double current_throttle,
                   const double current_steering,
                   const double current_steering_vel,
                   double& desired_throttle,
                   double& desired_steering,
                   double& desired_steering_vel,
                   double dt) const {

    // BEGIN THROTTLE LIMITATION SECTION
    // limit current_throttle to [min,max]
    if (desired_throttle > params_->throttle_max) {desired_throttle = params_->throttle_max;}
    if (desired_throttle < params_->throttle_min) {desired_throttle = params_->throttle_min;}
    // scale throttle to velocity commanded
    double new_velocity = throttleToSpeed(desired_throttle);
    if (new_velocity > params_->velocity_max) {
      new_velocity = params_->velocity_max;
      desired_throttle = speedToThrottle(new_velocity);
    }
    if (new_velocity < params_->velocity_min) {
      new_velocity = params_->velocity_min;
      desired_throttle = speedToThrottle(new_velocity);
    }

    // scale previous throttle to velocity & calculate commanded acceleration
    double current_velocity = throttleToSpeed(current_throttle);
    double desired_acceleration = (new_velocity - current_velocity) / dt;
    // limit acceleration
    if (desired_acceleration > params_->acceleration_max) {
      desired_acceleration = params_->acceleration_max;
      new_velocity = desired_acceleration * dt;
      desired_throttle = speedToThrottle(new_velocity);
    }
    if (desired_acceleration < params_->acceleration_min) {
      desired_acceleration = params_->acceleration_min;
      new_velocity = desired_acceleration * dt;
      desired_throttle = speedToThrottle(new_velocity);
      }
    // END THROTTLE LIMITATION SECTION

    // BEGIN STEERING LIMITATION SECTION
    double heading_vel;
    double heading_accel;
    // limit heading my max angle
    if ((desired_steering - current_steering) > params_->max_steering_angle) {
      desired_steering = params_->max_steering_angle;
    }
    if ((desired_steering - current_steering) < -params_->max_steering_angle) {
      desired_steering = -params_->max_steering_angle;
    }

    // limit heading by max angle rate of change (angular velocity)
    desired_steering_vel = (desired_steering - current_steering) / dt;
    if (desired_steering_vel > params_->angular_velocity_max) {
      desired_steering_vel = params_->angular_velocity_max;
      desired_steering = current_steering + desired_steering_vel*dt;
    }
    if (desired_steering_vel < params_->angular_velocity_min) {
      desired_steering_vel = params_->angular_velocity_min;
      desired_steering = current_steering + desired_steering_vel*dt;
    }

    // limit heading by max angle rate of change rate of change (angular accel)
    heading_accel = (desired_steering_vel - current_steering_vel) / dt;
    if (heading_accel > params_->angular_acceleration_max) {
      heading_accel = params_->angular_acceleration_max;
      desired_steering_vel = current_steering_vel + heading_accel*dt;
      desired_steering = current_steering + desired_steering_vel*dt;
    }
    if (heading_accel < params_->angular_acceleration_min) {
      heading_accel = params_->angular_acceleration_min;
      desired_steering_vel = current_steering_vel + heading_accel*dt;
      desired_steering = current_steering + desired_steering_vel*dt;
    }
}

} // namespace ackermann

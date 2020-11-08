/**
*  @file Limits.cpp
*
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Santosh Kesani
 *
 * @copyright [2020]
 */

#include <Limits.hpp>
#include <cmath>

namespace ackermann {

Limits::Limits(const std::shared_ptr<const Params>& params)
  : params_(params) {
}

double Limits::throttleToSpeed(double throttle) const {
  double speed_calc;
  if (throttle > params_->throttle_max)
    throttle = params_->throttle_max;
  if (throttle < params_->throttle_min)
    throttle = params_->throttle_min;

  if (throttle <= 0)
    speed_calc = 0;
  else
    speed_calc = throttle * params_->velocity_max;

  return speed_calc;
}

double Limits::speedToThrottle(double speed) const {
  double throttle_calc;

  if (speed > params_->velocity_max)
    speed = params_->velocity_max;
  if (speed < params_->velocity_min)
    speed = params_->velocity_min;

  if (speed <= 0)
    throttle_calc = 0;
  else
    throttle_calc = speed / params_->velocity_max;
  return throttle_calc;
}

double Limits::shortestArcToTurn(double current_heading,
                                 double desired_heading) const {
  double heading_command = (desired_heading - current_heading);
  if (heading_command > M_PI)
    heading_command -= 2*M_PI;
  if (heading_command < -M_PI)
    heading_command += 2*M_PI;
  return heading_command;
}

double Limits::boundHeading(const double heading) const {
  double temp_heading = heading;

  if (temp_heading < -M_PI)
    while (temp_heading < -M_PI)
      temp_heading += 2*M_PI;

  if (temp_heading >= M_PI)
    while (temp_heading > M_PI)
      temp_heading -= 2*M_PI;

  return temp_heading;
}

void Limits::limit(const double current_speed,
                   const double current_steering,
                   const double current_steering_vel,
                   double& desired_throttle,
                   double& desired_steering,
                   double& desired_steering_vel,
                   double dt) const {
    // BEGIN THROTTLE LIMITATION SECTION
    // limit current_throttle to [min,max]
    if (desired_throttle > params_->throttle_max)
      desired_throttle = params_->throttle_max;
    if (desired_throttle < params_->throttle_min)
      desired_throttle = params_->throttle_min;

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
    double desired_acceleration = (new_velocity - current_speed) / dt;
    // limit acceleration
    if (desired_acceleration > params_->acceleration_max) {
      desired_acceleration = params_->acceleration_max;
      new_velocity = current_speed + desired_acceleration * dt;
      desired_throttle = speedToThrottle(new_velocity);
    }
    if (desired_acceleration < params_->acceleration_min) {
      desired_acceleration = params_->acceleration_min;
      new_velocity = current_speed + desired_acceleration * dt;
      desired_throttle = speedToThrottle(new_velocity);
      }
    // END THROTTLE LIMITATION SECTION

    // BEGIN STEERING LIMITATION SECTION
    // limit heading my max angle
    if (desired_steering > params_->max_steering_angle) {
      desired_steering = params_->max_steering_angle;
    }
    if (desired_steering < -params_->max_steering_angle) {
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
    double steering_accel = (desired_steering_vel - current_steering_vel) / dt;
    if (steering_accel > params_->angular_acceleration_max) {
      steering_accel = params_->angular_acceleration_max;
      desired_steering_vel = current_steering_vel + steering_accel*dt;
      desired_steering = current_steering
                         + (current_steering_vel*dt)
                         + .5*steering_accel*dt*dt;
    }
    if (steering_accel < params_->angular_acceleration_min) {
      steering_accel = params_->angular_acceleration_min;
      desired_steering_vel = current_steering_vel + steering_accel*dt;
      desired_steering = current_steering
                         + (current_steering_vel*dt)
                         + .5*steering_accel*dt*dt;
    }
}

}  // namespace ackermann

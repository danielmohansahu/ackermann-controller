/* @file Model.cpp
 * @brief This is the class definition for a vehicle which
 * uses an Ackermann Steering Controller.
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 *
 * @copyright [2020]
 *
 * http://www.iaeng.org/publication/WCE2016/WCE2016_pp1062-1066.pdf
 * https://www.xarg.org/book/kinematics/ackerman-steering/
 */

#include <Model.hpp>

#include <math.h>

#include <iostream>
#include <atomic>

namespace ackermann {

Model::Model(const std::shared_ptr<const Params>& params)
  : params_(params) {
  this->limits_ = std::make_unique<Limits>(params);
  this->reset();
}

void Model::reset() {
  this->desired_speed_ = 0.0;
  this->desired_heading_ = 0.0;
  this->current_speed_ = 0.0;
  this->current_heading_ = 0.0;
  this->current_throttle_ = 0.0;
  this->current_steering_ = 0.0;
}

void Model::setState(const double speed, const double heading) {
  this->current_speed_ = speed;
  this->current_heading_ = limits_->boundHeading(heading);
}

void Model::getState(double& speed, double& heading) const {
  speed = this->current_speed_;
  heading = this->current_heading_;
}

void Model::setGoal(const double speed, const double heading) {
  this->desired_speed_ = speed;
  this->desired_heading_ = limits_->boundHeading(heading);
}

void Model::getGoal(double& speed, double& heading) const {
  speed = this->desired_speed_;
  heading = this->desired_heading_;
}

void Model::getCommand(double& throttle, double& steering) const {
  throttle = this->current_throttle_;
  steering = this->current_steering_;
}

void Model::getCommand(double& throttle, double& steering,
double& steer_vel) const {
  throttle = this->current_throttle_;
  steering = this->current_steering_;
  steer_vel = this->current_steering_vel_;
}

void Model::command(const double cmd_throttle, const double steering,
const double dt) {
  // update current throttle to new value
  this->current_throttle_ = cmd_throttle;
  // take throttle and convert to speed
  this->current_speed_ = limits_->throttleToSpeed(cmd_throttle);
  // update current steering value to output from limit
  this->current_steering_vel_ = (steering - this->current_steering_) / dt;
  this->current_steering_ = steering;
  // update current heading to new heading value
  this->current_heading_ = limits_->boundHeading(this->current_heading_ +
    ((this->current_speed_/params_->wheel_base) * tan(steering) * dt));
}

void Model::getError(double& speed_error, double& heading_error) const {
  speed_error = desired_speed_ - current_speed_;
  // minimize heading error
  heading_error = limits_->shortestArcToTurn(current_heading_,
    desired_heading_);
}

void Model::getWheelLinVel(double& wheel_LeftFront, double& wheel_RightFront,
  double& wheel_LeftRear, double& wheel_RightRear) const {
  // https://www.xarg.org/book/kinematics/ackerman-steering/

    // if no current steering input, then the radius of curvature is infinite;
    // this causes bad things if not caught
    if (this->current_steering_ != 0) {
      // use bicycle model for steering input (estimate single wheel in
      // front+center of rover)
      double turning_radius = params_->wheel_base /
        tan(this->current_steering_);
      // note: left turn == negative turning radius. All calculations hold
      // until linear velocity calculations

      // rear axle is aligned with radius of turning circle
      double radius_RR = turning_radius - (params_->track_width/2);
      double radius_LR = turning_radius + (params_->track_width/2);
      // front axle is not aligned; use Pythagoras to calculate radius
      double radius_RF = std::sqrt(std::pow(params_->wheel_base, 2) +
        std::pow(radius_RR, 2));
      double radius_LF = std::sqrt(std::pow(params_->wheel_base, 2) +
        std::pow(radius_LR, 2));

      // angular velocity calculation - done at center, since result holds
      // across any radius
      double curr_angular_vel = this->current_speed_ / turning_radius;
      // calculate linear velocities at each distance from wheel to center of
      // turning circle. Absolute value because turning_radius is negative
      // for a left turn, but wheels still moving forward.
      wheel_LeftRear = abs(curr_angular_vel * radius_LR);
      wheel_RightRear = abs(curr_angular_vel * radius_RR);
      wheel_LeftFront = abs(curr_angular_vel * radius_LF);
      wheel_RightFront = abs(curr_angular_vel * radius_RF);
    } else {
    // if no current steering, then driving straight, and all wheel speeds
    // equal current speed
      wheel_LeftRear = this->current_speed_;
      wheel_RightRear = this->current_speed_;
      wheel_LeftFront = this->current_speed_;
      wheel_RightFront = this->current_speed_;
    }
  }

}  // namespace ackermann

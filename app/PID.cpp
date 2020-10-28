/* @file PID.cpp
 * @brief PID controller
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Santosh Kesani
 * 
 * @copyright [2020]
 */

#include <PID.hpp>

namespace ackermann {

PID::PID(const std::shared_ptr<const PIDParams>& params)
  : params_(params),
    prev_error_(0.0),
    integral_error_(0.0) {
}

double PID::get_k_p() const {return this->params_->kp;}
double PID::get_k_i() const {return this->params_->ki;}
double PID::get_k_d() const {return this->params_->kd;}

double PID::getCommand(double current, double desired, double dt) {
  // // calculate current error
  // double current_error = desired - current;
  // // Integral controller portion
  // double integral = integral_error_ + (current_error * dt);
  // // Derivative
  // double derivative = (current_error - prev_error_) / dt;
  // // calculate output
  // double output = (k_p_*current_error) + (k_i_*integral) + (k_d_*derivative);
  // // save error as previous prev_error_
  // prev_error_ = current_error;
  // // save integral as integral error;
  // integral_error_ = integral;
  // return output;
  return 0.0;
}

} // namespace ackermann

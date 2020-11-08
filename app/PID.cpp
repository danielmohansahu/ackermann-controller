/* @file PID.cpp
 * @brief PID controller
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Santosh Kesani
 *
 * @copyright [2020]
 */
#include <iostream>
#include <PID.hpp>

namespace ackermann {

PID::PID(const std::shared_ptr<const PIDParams>& params,
         double out_minLimit,
         double out_maxLimit)
  : params_{params},
    prev_error_{0.0},
    integral_error_{0.0},
    out_minLimit_{out_minLimit},
    out_maxLimit_{out_maxLimit} {
}

double PID::get_k_p() const {return this->params_->kp;}
double PID::get_k_i() const {return this->params_->ki;}
double PID::get_k_d() const {return this->params_->kd;}

double PID::getCommand(double current_error, double dt) {
  // Integral controller portion
  integral_error_ += (current_error * dt);
  // Derivative
  double derivative = (current_error - prev_error_) / dt;
  // calculate output
  double output = (params_->kp*current_error)
                  + (params_->ki*integral_error_)
                  + (params_->kd*derivative);
  // PID windup
  if (output > out_maxLimit_) {
    integral_error_ -= output - out_maxLimit_;
    output = out_maxLimit_;
  } else if (output < out_minLimit_) {
      integral_error_ += out_minLimit_ - output;
      output = out_minLimit_;
  }
  // save error as previous prev_error_
  prev_error_ = current_error;
  // return output;
  return output;
}

void PID::reset_PID() {
    this->prev_error_ = 0.0;
    this->integral_error_ = 0.0;
}

}  // namespace ackermann

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

namespace ackermann
{

PID::PID(double k_p, double k_i, double k_d)
 : k_p_(k_p),
   k_i_(k_i),
   k_d_(k_d),
   prev_error_(0.0),
   integral_error_(0.0)
{
}

void PID::set_k_p(double kp) {this->k_p_ = kp;}
void PID::set_k_i(double ki) {this->k_i_ = ki;}
void PID::set_k_d(double kd) {this->k_d_ = kd;}

double PID::get_k_p() const {return this->k_p_;}
double PID::get_k_i() const {return this->k_i_;}
double PID::get_k_d() const {return this->k_d_;}

double PID::compute(double current, double desired)
{
  // // calculate current error
  // double current_error = desired - current;
  // // Integral controller portion
  // double integral = integral_error_ + (current_error * global_dt);
  // // Derivative
  // double derivative = (current_error - prev_error_) / global_dt;
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
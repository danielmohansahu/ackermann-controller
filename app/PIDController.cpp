/* @file PIDController.cpp
 * @brief PID controller using implementation from previous week
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @copyright [2020]
 */

 #include <PIDController.hpp>

 ControlAlgo::PIDController::PIDController(double k_p, double k_i, double k_d, double dt) {
   k_p_ = k_p;
   k_i_ = k_i;
   k_d_ = k_d;
   dt_ = dt;
   prev_error_ = 0;
   integral_error_ = 0;
 }

void ControlAlgo::PIDController::set_k_p(double kp) {this->k_p_ = kp;}
void ControlAlgo::PIDController::set_k_i(double ki) {this->k_i_ = ki;}
void ControlAlgo::PIDController::set_k_d(double kd) {this->k_d_ = kd;}

double ControlAlgo::PIDController::get_k_p_() const {return this->k_p_;}
double ControlAlgo::PIDController::get_k_i_() const {return this->k_i_;}
double ControlAlgo::PIDController::get_k_d_() const {return this->k_d_;}

double ControlAlgo::PIDController::Compute(double current, double desired) {
  // calculate current error
  double current_error = desired - current;
  // Integral controller portion
  double integral = integral_error_ + (current_error * dt_);
  // Derivative
  double derivative = (current_error - prev_error_) / dt_;
  // calculate output
  double output = (k_p_*current_error) + (k_i_*integral) + (k_d_*derivative);
  // save error as previous prev_error_
  prev_error_ = current_error;
  // save integral as integral error;
  integral_error_ = integral;
  return output;
}

// destructor
ControlAlgo::PIDController::~PIDController() {}

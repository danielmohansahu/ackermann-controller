#pragma once

/* @file Vehicle.hpp
 * @brief Class declaration for a PID controller
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @copyright [2020]
 */

#include <iostream>

namespace ControlAlgo {
  class PIDController{
  public:
    PIDController(double k_p_ = 0., double k_i_ = 0., double k_d_ = 0., double dt_ = 0.);
    void set_k_p(double);
    void set_k_i(double);
    void set_k_d(double);
    double get_k_p_() const;
    double get_k_i_() const;
    double get_k_d_() const;
    double Compute(double, double);
    virtual ~PIDController();
  private:
    double k_p_;
    double k_i_;
    double k_d_;
    double dt_;
    double prev_error_;
    double integral_error_;
  };
}

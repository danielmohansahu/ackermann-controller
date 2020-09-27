#pragma once

/* @file controller.hpp
 * @brief Class declaration for an Ackermann Controller.
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @copyright [2020]
 */

#include <iostream>

class AckermannController
{
  public:
    // @brief constructor
    AckermannController();

    // @brief core computation method
    double compute(double heading, double vel);

  private:
    double steering_constraint_;
};

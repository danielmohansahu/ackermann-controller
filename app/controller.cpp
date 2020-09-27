/* @file controller.cpp
 * @brief This is the class definition for an Ackermann Steering Controller.
 * 
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 *
 * @copyright [2020]
 */ 

#include <controller.hpp>

AckermannController::AckermannController()
    : steering_constraint_(45.0)
{
}

// stub implementation
double AckermannController::compute(double heading, double velocity)
{
    return 0.0;
}

/* @file plant.cpp
 */

#include "plant.h"

namespace fake
{

void Plant::setState(double speed, double heading)
{
  speed_ = speed;
  heading_ = heading;
}

void Plant::getState(double& speed, double& heading)
{
}

void Plant::applyCommand(double speed, double heading, double dt)
{
}

} // namespace fake
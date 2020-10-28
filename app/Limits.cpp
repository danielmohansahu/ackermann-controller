/* @file Limits.cpp
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Sentosh Kesani
 *
 * @copyright [2020]
 */

// @TODO Currently STUB implementation; needs to be filled

#include <Limits.hpp>

namespace ackermann {

Limits::Limits(const std::shared_ptr<const Params>& params)
  : params_(params) {
}

void Limits::limit(const double current_throttle,
                   const double current_steering,
                   double& desired_throttle,
                   double& desired_steering,
                   double dt) const {

}

} // namespace ackermann

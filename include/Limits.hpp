#pragma once
/* @file Limits.hpp
 * @copyright [2020]
 */

#include <limits>
#include "State.hpp"

namespace ackermann {

/* @brief Class used to apply known limits to a given desired control signal.
 */
class Limits {
  using max_val = std::numeric_limits<double>;

 public:
  /* @brief Constructor
   */
  explicit Limits(double velocity = max_val::max(),
                  double acceleration = max_val::max())
    : velocity_(velocity), acceleration_(acceleration) {
  }

  /* @brief Apply known limits to the given controller command.
  */
  State limit(const State& current, const State& desired, double dt);

 private:
  double velocity_;
  double acceleration_;
};

} // namespace ackermann


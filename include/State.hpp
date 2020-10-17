#pragma once
/* @file State.hpp
 * @copyright [2020]
 */

namespace ackermann {

/* @brief Struct containing Ackermann State information.
 * 
 * Currently only contains left and right wheel speed information,
 * but is intended to be extensible.
 */
struct State {
  double left {0.0};
  double right {0.0};

  // empty constructor
  State() {}

  // @brief constructor with values
  State(double left_, double right_)
    : left(left_), right(right_){}
};

} // namespace ackermann


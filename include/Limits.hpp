#pragma once

#include <limits>
#include "State.hpp"

namespace ackermann
{

/* @brief Class used to apply known limits to a given desired control signal.
 */
class Limits
{
  using max_val = std::numeric_limits<double>;

 public:
  /* @brief Constructor
   */
  Limits()
   : velocity_(max_val::max()), acceleration_(max_val::max())
  {}

  /* @brief Apply known limits to the given controller command.
  */
  void limit(State& current, State& desired);

 private:
  double velocity_;
  double acceleration_;
};

} // namespace ackermann


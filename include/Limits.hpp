#pragma once

#include <limits>
#include "State.hpp"

namespace ackermann
{

/* @brief Class used to apply known limits to a given desired control signal.
 */
class Limits
{
  using max_value = std::numeric_limits<double>::max;

 public:
  /* @brief Constructor
   */
  Limits()
   : velocity_(max_value), acceleration_(max_value))
  {}

  /* @brief Apply known limits to the given controller command.
  */
  limit(State& current, State& desired);

 private:
  velocity_;
  acceleration_;
};

} // namespace ackermann


/* @file Limits.cpp
 * @copyright [2020]
 */

#include <gtest/gtest.h>
#include <Limits.hpp>
#include <Params.hpp>

using ackermann::Limits;
using ackermann::Params;

/* @brief Test limiting without any limits (should be default behavior).*/
TEST(Limits_NoLimit, should_pass) {
  double dt = 0.01;
  auto p = std::make_shared<Params>(0.0, 0.0, 0.0, 0.0);
  Limits lim(p);

  {
    // check #1: we don't limit 0 moves
    double original_throttle = 0.0;
    double original_steering = 0.0;
    double desired_throttle = original_throttle;
    double desired_steering = original_steering;
    lim.limit(0.0, 0.0, desired_throttle, desired_steering, dt);
    EXPECT_EQ(original_throttle, desired_throttle);
    EXPECT_EQ(original_steering, desired_steering);
  }

  {
    // check #2: we shouldn't limit arbitrary values
    double original_throttle = 1000000;
    double original_steering = 1000000;
    double desired_throttle = original_throttle;
    double desired_steering = original_steering;
    lim.limit(0.0, 0.0, desired_throttle, desired_steering, dt);
    EXPECT_EQ(original_throttle, desired_throttle);
    EXPECT_EQ(original_steering, desired_steering);
  }

  {
    // check #3: make sure negatives work properly too
    double original_throttle = -1000000;
    double original_steering = -1000000;
    double desired_throttle = original_throttle;
    double desired_steering = original_steering;
    lim.limit(0.0, 0.0, desired_throttle, desired_steering, dt);
    EXPECT_EQ(original_throttle, desired_throttle);
    EXPECT_EQ(original_steering, desired_steering);
  }
}

/* @brief Test velocity limiting.*/
TEST(Limits_Velocity, should_pass) {
  // instantiate a limits class with only velocity limits
  double dt = 0.01;
  auto p = std::make_shared<Params>(0.0, 0.0, 0.0, 0.0);
  p->velocity_max = 1.0;
  p->velocity_min = 0.0;
  Limits lim(p);

  {
    // check #1: make sure we throttle velocity (only!)
    double original_throttle = 1000000;
    double original_steering = 1000000;
    double desired_throttle = original_throttle;
    double desired_steering = original_steering;
    lim.limit(0.0, 0.0, desired_throttle, desired_steering, dt);
    EXPECT_EQ(desired_throttle, p->velocity_max);
    EXPECT_EQ(original_steering, desired_steering);    
  }

  {
    // check #2: make sure negatives work properly too
    double original_throttle = -1000000;
    double original_steering = -1000000;
    double desired_throttle = original_throttle;
    double desired_steering = original_steering;
    lim.limit(0.0, 0.0, desired_throttle, desired_steering, dt);
    EXPECT_EQ(desired_throttle, p->velocity_min);
    EXPECT_EQ(original_steering, desired_steering);    
  }
}

/* @brief Test acceleration limiting.*/
TEST(Limits_Acceleration, should_pass) {
  // instantiate a limits class with only acceleration limits
  double dt = 0.01;
  auto p = std::make_shared<Params>(0.0, 0.0, 0.0, 0.0);
  p->acceleration_min = -1.0;
  p->acceleration_max = 1.0;
  Limits lim(p);

  {
    double original_throttle = 1000000;
    double original_steering = 1000000;
    double desired_throttle = original_throttle;
    double desired_steering = original_steering;
    lim.limit(0.0, 0.0, desired_throttle, desired_steering, dt);
    EXPECT_EQ(desired_throttle, p->acceleration_max * dt);
    EXPECT_EQ(original_steering, desired_steering);
  }

  {
    // check #2: make sure negatives work properly too
    double original_throttle = -1000000;
    double original_steering = -1000000;
    double desired_throttle = original_throttle;
    double desired_steering = original_steering;
    lim.limit(0.0, 0.0, desired_throttle, desired_steering, dt);
    EXPECT_EQ(desired_throttle, p->acceleration_min * dt);
    EXPECT_EQ(original_steering, desired_steering);
  }
}

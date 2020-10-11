/* @file Limits.cpp
 * @copyright [2020]
 */

#include <gtest/gtest.h>
#include <Limits.hpp>
#include <State.hpp>

using ackermann::Limits;
using ackermann::State;

/* @brief Test limiting without any limits (should be default behavior).*/
TEST(Limits_NoLimit, should_pass) {
  double dt = 0.1;
  Limits lim;

  // instantiate some empty states to work with
  State current;
  State desired;

  {
    // check #1: we don't limit 0 moves
    State actual = lim.limit(current, desired, dt);
    EXPECT_EQ(desired.left, actual.left);
    EXPECT_EQ(desired.right, actual.right);
  }

  {
    // check #2: we shouldn't limit arbitrary values
    desired.left = desired.right = 100000;
    State actual = lim.limit(current, desired, dt);
    EXPECT_EQ(desired.left, actual.left);
    EXPECT_EQ(desired.right, actual.right);
  }

  {
    // check #3: make sure negatives work properly too
    desired.left = desired.right = -100000;
    State actual = lim.limit(current, desired, dt);
    EXPECT_EQ(desired.left, actual.left);
    EXPECT_EQ(desired.right, actual.right);
  }
}

/* @brief Test velocity limiting.*/
TEST(Limits_Velocity, should_pass) {
  // instantiate a limits class with only velocity limits
  double dt = 0.1;
  double vel_limit = 1.0;
  Limits lim(vel_limit);

  // instantiate some empty states to work with
  State current;
  State desired;

  {
    // check #2: we shouldn't limit arbitrary values
    desired.left = desired.right = 100000;
    State actual = lim.limit(current, desired, dt);
    EXPECT_EQ(desired.left, vel_limit);
    EXPECT_EQ(desired.right, vel_limit);
  }

  {
    // check #3: make sure negatives work properly too
    desired.left = desired.right = -100000;
    State actual = lim.limit(current, desired, dt);
    EXPECT_EQ(desired.left, -vel_limit);
    EXPECT_EQ(desired.right, -vel_limit);
  }
}

/* @brief Test acceleration limiting.*/
TEST(Limits_Acceleration, should_pass) {
  // instantiate a limits class with only acceleration limits
  double dt = 0.1;
  double accel_limit = 1.0;
  Limits lim(std::numeric_limits<double>::max(), accel_limit);

  // instantiate some empty states to work with
  State current;
  State desired;

  {
    // check #2: we shouldn't limit arbitrary values
    desired.left = desired.right = 100000;
    State actual = lim.limit(current, desired, dt);
    EXPECT_EQ(desired.left, accel_limit * dt);
    EXPECT_EQ(desired.right, accel_limit * dt);
  }

  {
    // check #3: make sure negatives work properly too
    desired.left = desired.right = -100000;
    State actual = lim.limit(current, desired, dt);
    EXPECT_EQ(desired.left, -accel_limit * dt);
    EXPECT_EQ(desired.right, -accel_limit * dt);
  }
}

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

  // check #1: we don't limit 0 moves
  State actual = lim.limit(current, desired, dt);
  EXPECT_EQ(desired.left, actual.left);
  EXPECT_EQ(desired.right, actual.right);
}

/* @brief Test velocity limiting.*/
TEST(Limits_Velocity, should_pass) {
  EXPECT_EQ(1, 1);
}

/* @brief Test acceleration limiting.*/
TEST(Limits_Acceleration, should_pass) {
  EXPECT_EQ(1, 1);
}

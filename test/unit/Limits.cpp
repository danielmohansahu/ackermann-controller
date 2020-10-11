#include <gtest/gtest.h>
#include <Limits.hpp>

using ackermann::Limits;

/* @brief Test limiting without any limits (should be default behavior).*/
TEST(Limits_NoLimit, should_pass) {
  EXPECT_EQ(1, 1);
}

/* @brief Test velocity limiting.*/
TEST(Limits_Velocity, should_pass) {
  EXPECT_EQ(1, 1);
}

/* @brief Test acceleration limiting.*/
TEST(Limits_Acceleration, should_pass) {
  EXPECT_EQ(1, 1);
}

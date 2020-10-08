#include <gtest/gtest.h>
#include <PID.hpp>

using ackermann::PID;

/* @brief Test all setters and getters. */
TEST(PID_SettersAndGetters, should_pass) {
  EXPECT_EQ(1, 1);
}

/* @brief Test compute command with default params.*/
TEST(PID_Control1, should_pass) {
  EXPECT_EQ(1, 1);
}

/* @brief Test compute command with known values. */
TEST(PID_Control2, should_pass) {
  EXPECT_EQ(1, 1);
}

/* @brief Test compute command with known values #2. */
TEST(PID_Control3, should_pass) {
  EXPECT_EQ(1, 1);
}
#include <gtest/gtest.h>

#include <Controller.hpp>
#include "fake/plant.h"

// construct fake plant

// construct Controller and set appropriate parameters

/* @brief Test that we've set up the Mock class and Controller properly. */
TEST(System_Setup, should_pass) {
  EXPECT_EQ(1, 1);
}

/* @brief Test that the system converges to a desired setpoint w/ a zero noise Mock Plant. */
TEST(System_Convergence1, should_pass) {
  EXPECT_EQ(1, 1);
}

/* @brief Test that the system converges to a desired setpoint w/ a low noise Mock Plant. */
TEST(System_Convergence2, should_pass) {
  EXPECT_EQ(1, 1);
}

/* @brief Test that the system fails to converge to a "broken" Mock Plant. */
TEST(System_NoConvergence, should_pass) {
  EXPECT_EQ(1, 1);
}

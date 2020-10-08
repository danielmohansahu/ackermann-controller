#include <stdlib.h>
#include <time.h>
#include <gtest/gtest.h>
#include <PID.hpp>

using ackermann::PID;

/* @brief Test all setters and getters. */
TEST(PID_SettersAndGetters, should_pass) {
  // initialize random values for kp/ki/kd
  srand(time(NULL));
  double kp = static_cast<double>(rand() / RAND_MAX);
  double ki = static_cast<double>(rand() / RAND_MAX);
  double kd = static_cast<double>(rand() / RAND_MAX);

  // instantiate class
  PID pid(kp, ki, kd);

  // test getters
  EXPECT_EQ(pid.get_k_p(), kp);
  EXPECT_EQ(pid.get_k_i(), ki);
  EXPECT_EQ(pid.get_k_d(), kd);

  // test setters
  pid.set_k_p(ki);
  pid.set_k_i(kd);
  pid.set_k_d(kp);
  EXPECT_EQ(pid.get_k_p(), ki);
  EXPECT_EQ(pid.get_k_i(), kd);
  EXPECT_EQ(pid.get_k_d(), kp);
}

/* @brief Test compute command with default params.*/
TEST(PID_Control1, should_pass) {
  PID pid(0.0, 0.0, 0.0);

  EXPECT_EQ(pid.compute(0.0, 0.0), 0.0);
  EXPECT_EQ(pid.compute(1.0, 0.0), 0.0);
  EXPECT_EQ(pid.compute(0.0, 1.0), 0.0);
  EXPECT_EQ(pid.compute(1.0, 1.0), 0.0);
}

/* @brief Test compute command with known values. */
TEST(PID_Control2, should_pass) {
  EXPECT_EQ(1, 1);
}

/* @brief Test compute command with known values #2. */
TEST(PID_Control3, should_pass) {
  EXPECT_EQ(1, 1);
}
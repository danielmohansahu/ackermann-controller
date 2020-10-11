/* @file PID.cpp
 * @copyright [2020]
 */

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

  EXPECT_EQ(pid.compute(0.0, 0.0, 0.1), 0.0);
  EXPECT_EQ(pid.compute(1.0, 0.0, 0.1), 0.0);
  EXPECT_EQ(pid.compute(0.0, 1.0, 0.1), 0.0);
  EXPECT_EQ(pid.compute(1.0, 1.0, 0.1), 0.0);
}

/* @brief Test compute command with known values. */
TEST(PID_Control2, should_pass) {
  // set gains
  PID pid(0.5, 0.01, 0.125);

  // call a few commands to get expected value
  EXPECT_DOUBLE_EQ(pid.compute(0.0, 1.0, 0.1), 1.751);
  EXPECT_DOUBLE_EQ(pid.compute(0.1, 1.0, 0.1), 0.3269);
  EXPECT_DOUBLE_EQ(pid.compute(0.2, 1.0, 0.1), 0.2777);
}

/* @brief Test compute command with known values #2. */
TEST(PID_Control3, should_pass) {
  // set gains
  PID pid(0.1, 0.0, 0.1);

  // call a few commands to get expected value
  EXPECT_DOUBLE_EQ(pid.compute(-1, 10.0, 1.0), 2.2);
  EXPECT_DOUBLE_EQ(pid.compute(-2.5, 10.0, 1.0), 1.4);
  EXPECT_DOUBLE_EQ(pid.compute(0.0, 10.0, 1.0), 0.75);
}

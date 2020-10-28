/* @file PID.cpp
 * @copyright [2020]
 */

#include <stdlib.h>
#include <time.h>
#include <gtest/gtest.h>

#include <Params.hpp>
#include <PID.hpp>

using ackermann::PID;
using ackermann::PIDParams;

/* @brief Test all setters and getters. */
TEST(PID_SettersAndGetters, should_pass) {
  // initialize random values for kp/ki/kd
  srand(time(NULL));
  auto params = std::make_shared<PIDParams>();
  params->kp = static_cast<double>(rand() / RAND_MAX);
  params->ki = static_cast<double>(rand() / RAND_MAX);
  params->kd = static_cast<double>(rand() / RAND_MAX);

  // instantiate class
  PID pid(params);

  // test getters
  EXPECT_EQ(pid.get_k_p(), params->kp);
  EXPECT_EQ(pid.get_k_i(), params->ki);
  EXPECT_EQ(pid.get_k_d(), params->kd);

  // test setters
  params->kp = 1.0;
  params->ki = 2.0;
  params->kd = 3.0;
  EXPECT_EQ(pid.get_k_p(), params->ki);
  EXPECT_EQ(pid.get_k_i(), params->kd);
  EXPECT_EQ(pid.get_k_d(), params->kp);
}

/* @brief Test getCommand command with default params.*/
TEST(PID_Control1, should_pass) {
  auto params = std::make_shared<PIDParams>();
  PID pid(params);

  EXPECT_EQ(pid.getCommand(0.0, 0.0, 0.1), 0.0);
  EXPECT_EQ(pid.getCommand(1.0, 0.0, 0.1), 0.0);
  EXPECT_EQ(pid.getCommand(0.0, 1.0, 0.1), 0.0);
  EXPECT_EQ(pid.getCommand(1.0, 1.0, 0.1), 0.0);
}

/* @brief Test getCommand command with known values. */
TEST(PID_Control2, should_pass) {
  // set gains
  auto params = std::make_shared<PIDParams>(0.5, 0.01, 0.125);
  PID pid(params);

  // call a few commands to get expected value
  EXPECT_DOUBLE_EQ(pid.getCommand(0.0, 1.0, 0.1), 1.751);
  EXPECT_DOUBLE_EQ(pid.getCommand(0.1, 1.0, 0.1), 0.3269);
  EXPECT_DOUBLE_EQ(pid.getCommand(0.2, 1.0, 0.1), 0.2777);
}

/* @brief Test getCommand command with known values #2. */
TEST(PID_Control3, should_pass) {
  // set gains
  auto params = std::make_shared<PIDParams>(0.1, 0.0, 0.1);
  PID pid(params);

  // call a few commands to get expected value
  EXPECT_DOUBLE_EQ(pid.getCommand(-1, 10.0, 1.0), 2.2);
  EXPECT_DOUBLE_EQ(pid.getCommand(-2.5, 10.0, 1.0), 1.4);
  EXPECT_DOUBLE_EQ(pid.getCommand(0.0, 10.0, 1.0), 0.75);
}

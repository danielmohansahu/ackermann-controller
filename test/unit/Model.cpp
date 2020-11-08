/* @file Ackermann.cpp
 * @copyright [2020]
 */

#include <gtest/gtest.h>

#include <iostream>
#include <cmath>

#include "Model.hpp"
#include "Limits.hpp"
#include "Params.hpp"


/* @brief Test Fixture for repeated independent construction of the Model. */
class AckemannModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // construct our base classes;
    params_ = std::make_shared<ackermann::Params>(1.0, 2.0, 0.785, 1.0, 1.0);
    params_->velocity_max = 10.0;
    params_->velocity_min = -1.0;
    params_->acceleration_max = 100;
    params_->acceleration_min = -100;
    model_ = std::make_unique<ackermann::Model>(params_);
    limits_ = std::make_unique<ackermann::Limits>(params_);
  }

  // our base class members (we use pointers to avoid default construction)
  std::unique_ptr<ackermann::Model> model_;
  std::unique_ptr<ackermann::Limits> limits_;
  std::shared_ptr<ackermann::Params> params_;
};

/* @brief Test all setters and Getters. */
TEST_F(AckemannModelTest, Model_SettersAndGetters) {
  // test setting and getting state
  {
    double speed, heading, speed_out, heading_out;
    // default state should be 0.0
    model_->getState(speed, heading);
    EXPECT_DOUBLE_EQ(speed, 0.0);
    EXPECT_DOUBLE_EQ(heading, 0.0);

    // set to arbitrary value and make sure it actually happens
    speed = 2.0;
    heading = 0.35;
    model_->setState(speed, heading);
    model_->getState(speed_out, heading_out);
    EXPECT_DOUBLE_EQ(speed, speed_out);
    EXPECT_DOUBLE_EQ(heading, heading_out);
  }

  // test reset
  {
    double speed, heading;
    model_->getState(speed, heading);
    EXPECT_NE(speed, 0);
    EXPECT_NE(heading, 0);
    model_->reset();
    model_->getState(speed, heading);
    EXPECT_DOUBLE_EQ(speed, 0);
    EXPECT_DOUBLE_EQ(heading, 0);
  }
  // test setting and getting state
  {
    double speed, heading, speed_out, heading_out;
    model_->reset();
    // default state should be 0.0
    model_->getState(speed, heading);
    EXPECT_DOUBLE_EQ(speed, 0.0);
    EXPECT_DOUBLE_EQ(heading, 0.0);

    // set to arbitrary value and make sure it actually happens
    speed = 2.0;
    heading = -0.35;
    model_->setState(speed, heading);
    model_->getState(speed_out, heading_out);
    EXPECT_DOUBLE_EQ(speed, speed_out);
    EXPECT_DOUBLE_EQ(heading, heading_out);
  }

  // test setting and getting command
  {
    double throttle, steering, throttle_out, steering_out;
    model_->reset();
    // default goal should be 0.0
    model_->getCommand(throttle, steering);
    EXPECT_DOUBLE_EQ(throttle, 0.0);
    EXPECT_DOUBLE_EQ(steering, 0.0);

    // set to arbitrary value and make sure it actually happens
    throttle = 0.25;
    steering = 0.7854;
    model_->command(throttle, steering, 0.01);
    model_->getCommand(throttle_out, steering_out);
    EXPECT_DOUBLE_EQ(throttle, throttle_out);
    EXPECT_DOUBLE_EQ(steering, steering_out);
  }
}


/* @brief Test error calculation. */
TEST_F(AckemannModelTest, Model_Error) {
  // test error #1: by default everything should be 0.0
  {
    // set state and goal
    model_->setState(0.0, 0.0);
    model_->setGoal(0.0, 0.0);

    double speed_error, heading_error;
    model_->getError(speed_error, heading_error);
    EXPECT_DOUBLE_EQ(speed_error, 0.0);
    EXPECT_DOUBLE_EQ(heading_error, 0.0);
  }

  // test error #2: calculate expected error
  {
    // set state and goal
    model_->setState(1.0, 0.0);
    model_->setGoal(2.0, 0.4);

    double speed_error, heading_error;
    model_->getError(speed_error, heading_error);
    EXPECT_DOUBLE_EQ(speed_error, 1.0);
    EXPECT_DOUBLE_EQ(heading_error, 0.4);
  }

  // test error #3: calculate expected error
  {
    // set state and goal
    model_->setState(8.0, -(M_PI/2 + 0.1));
    model_->setGoal(2.0, M_PI/2);

    double speed_error, heading_error;
    model_->getError(speed_error, heading_error);
    EXPECT_DOUBLE_EQ(speed_error, -6.0);
    EXPECT_DOUBLE_EQ(heading_error, -(M_PI - 0.1));
  }

// test error #3: calculate expected error, opposite direction
  {
    // set state and goal
    model_->setState(2.0, M_PI/2);
    model_->setGoal(8.0, -(M_PI/2 + 0.1));

    double speed_error, heading_error;
    model_->getError(speed_error, heading_error);
    EXPECT_DOUBLE_EQ(speed_error, 6.0);
    EXPECT_DOUBLE_EQ(heading_error, (M_PI - 0.1));
  }
}

/* @brief Test command execution. */
TEST_F(AckemannModelTest, Model_Command) {
  // test error #1: commanding a 0.0 move should set state to 0.0
  {
    // set goal
    model_->setState(0.0, 0.0);
    model_->command(0.0, 0.0, std::numeric_limits<double>::max());

    double speed, heading;
    model_->getState(speed, heading);
    EXPECT_DOUBLE_EQ(speed, 0.0);
    EXPECT_DOUBLE_EQ(heading, 0.0);
  }

  // test error #2: commanding a 0.0 move for infinite time should always
  // should set speed to 0.0
  {
    // set goal
    model_->setState(100.0, 0.0);
    model_->command(0.0, 0.0, std::numeric_limits<double>::max());

    double speed, heading;
    model_->getState(speed, heading);
    EXPECT_DOUBLE_EQ(speed, 0.0);
    EXPECT_DOUBLE_EQ(heading, 0.0);
  }

  // test error #3: commanding a 0.0 move for infinite time should always
  // should set speed to 0.0
  {
    // set goal
    model_->setState(-100.0, 0.0);
    model_->command(0.0, 0.0, std::numeric_limits<double>::max());

    double speed, heading;
    model_->getState(speed, heading);
    EXPECT_DOUBLE_EQ(speed, 0.0);
    EXPECT_DOUBLE_EQ(heading, 0.0);
  }
}

/* @brief Test command execution. */
TEST_F(AckemannModelTest, WheelSpeedCalc) {
  // test error #1: zero speed, zero steering
  {
    // set goal
    double speed_cmd = 0.0;
    double steering_input = 0.0;

    model_->setState(speed_cmd, 0.0);
    model_->command(limits_->speedToThrottle(speed_cmd), steering_input, 1);

    double LF, RF, LR, RR;
    model_->getWheelLinVel(LF, RF, LR, RR);
    EXPECT_DOUBLE_EQ(LF, speed_cmd);
    EXPECT_DOUBLE_EQ(RF, speed_cmd);
    EXPECT_DOUBLE_EQ(LR, speed_cmd);
    EXPECT_DOUBLE_EQ(RR, speed_cmd);
  }

  // test error #2: positive speed, zero steering
  {
    // set goal
    double speed_cmd = 2.0;
    double steering_input = 0.0;

    model_->setState(speed_cmd, 0.0);
    model_->command(limits_->speedToThrottle(speed_cmd), steering_input, 1);

    double LF, RF, LR, RR;
    model_->getWheelLinVel(LF, RF, LR, RR);
    EXPECT_DOUBLE_EQ(LF, speed_cmd);
    EXPECT_DOUBLE_EQ(RF, speed_cmd);
    EXPECT_DOUBLE_EQ(LR, speed_cmd);
    EXPECT_DOUBLE_EQ(RR, speed_cmd);
  }

  // test error #3: positive speed, right turn
  {
    // set goal
    double speed_cmd = 2.0;
    double steering_input = (M_PI/4);

    model_->setState(speed_cmd, 0.0);
    model_->command(limits_->speedToThrottle(speed_cmd), steering_input, 1);

    double LF, RF, LR, RR;
    model_->getWheelLinVel(LF, RF, LR, RR);
    EXPECT_NEAR(RF, 2, 1E-10);
    EXPECT_NEAR(LF, 2*std::sqrt(5), 1E-10);
    EXPECT_NEAR(LR, 4, 1E-10);
    EXPECT_NEAR(RR, 0, 1E-10);
  }

  // test error #3: positive speed, left turn
  {
    // set goal
    double speed_cmd = 2.0;
    double steering_input = -(M_PI/4);

    model_->setState(speed_cmd, 0.0);
    model_->command(limits_->speedToThrottle(speed_cmd), steering_input, 1);

    double LF, RF, LR, RR;
    model_->getWheelLinVel(LF, RF, LR, RR);
    EXPECT_NEAR(LF, 2, 1E-10);
    EXPECT_NEAR(RF, 2*std::sqrt(5), 1E-10);
    EXPECT_NEAR(RR, 4, 1E-10);
    EXPECT_NEAR(LR, 0, 1E-10);
  }
}

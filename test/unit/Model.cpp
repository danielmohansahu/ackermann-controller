/* @file Ackermann.cpp
 * @copyright [2020]
 */

#include <gtest/gtest.h>
#include "Model.hpp"
#include "Limits.hpp"
#include "Model.hpp"
#include "Params.hpp"
#include <iostream>

/* @brief Test Fixture for repeated independent construction of the Model. */
class AckemannModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // construct our base classes;
    params_ = std::make_shared<ackermann::Params>(1.0, 2.0, 0.0, 0.0);
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
    EXPECT_EQ(speed, 0.0);
    EXPECT_EQ(heading, 0.0);

    // set to arbitrary value and make sure it actually happens
    speed = 2.0;
    heading = -21.0;
    model_->setState(speed, heading);
    model_->getState(speed_out, heading_out);
    EXPECT_EQ(speed, speed_out);
    EXPECT_EQ(heading, heading_out);
  }

  // test setting and getting goal
  {
    double speed, heading, speed_out, heading_out;

    // default goal should be 0.0
    model_->getGoal(speed, heading);
    EXPECT_EQ(speed, 0.0);
    EXPECT_EQ(heading, 0.0);

    // set to arbitrary value and make sure it actually happens
    speed = 2.0;
    heading = -21.0;
    model_->setGoal(speed, heading);
    model_->getGoal(speed_out, heading_out);
    EXPECT_EQ(speed, speed_out);
    EXPECT_EQ(heading, heading_out);
  }

  // test setting and getting command
  {
    double throttle, steering, throttle_out, steering_out;

    // default goal should be 0.0
    model_->getCommand(throttle, steering);
    EXPECT_EQ(throttle, 0.0);
    EXPECT_EQ(steering, 0.0);

    // set to arbitrary value and make sure it actually happens
    throttle = 0.25;
    steering = 0.7854;
    model_->command(throttle, steering, 0.01);
    model_->getCommand(throttle_out, steering_out);
    EXPECT_EQ(throttle, throttle_out);
    EXPECT_EQ(steering, steering_out);
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
    EXPECT_EQ(speed_error, 0.0);
    EXPECT_EQ(heading_error, 0.0);
  }

  // test error #2: calculate expected error
  {
    // set state and goal
    model_->setState(1.0, 0.0);
    model_->setGoal(2.0, 25.0);

    double speed_error, heading_error;
    model_->getError(speed_error, heading_error);
    EXPECT_EQ(speed_error, 1.0);
    EXPECT_EQ(heading_error, 25.0);
  }

  // test error #3: calculate expected error
  {
    // set state and goal
    model_->setState(10.0, 2.0);
    model_->setGoal(2.0, 1.5);

    double speed_error, heading_error;
    model_->getError(speed_error, heading_error);
    EXPECT_EQ(speed_error, -8.0);
    EXPECT_EQ(heading_error,-0.5);
  }

// test error #3: calculate expected error, opposite direction
  {
    // set state and goal
    model_->setState(2.0, -1.5);
    model_->setGoal(8.0, 2);

    double speed_error, heading_error;
    model_->getError(speed_error, heading_error);
    EXPECT_EQ(speed_error, 6.0);
    EXPECT_EQ(heading_error, 3.5);
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
    EXPECT_EQ(speed, 0.0);
    EXPECT_EQ(heading, 0.0);
  }

  // test error #2: commanding a 0.0 move for infinite time should always should set speed to 0.0
  {
    // set goal
    model_->setState(100.0, 0.0);
    model_->command(0.0, 0.0, std::numeric_limits<double>::max());

    double speed, heading;
    model_->getState(speed, heading);
    EXPECT_EQ(speed, 0.0);
    EXPECT_EQ(heading, 0.0);
  }

  // test error #3: commanding a 0.0 move for infinite time should always should set speed to 0.0
  {
    // set goal
    model_->setState(-100.0, 0.0);
    model_->command(0.0, 0.0, std::numeric_limits<double>::max());

    double speed, heading;
    model_->getState(speed, heading);
    EXPECT_EQ(speed, 0.0);
    EXPECT_EQ(heading, 0.0);
  }
}

#include <iostream>
#include <gtest/gtest.h>

#include <Controller.hpp>
#include "fake/plant.h"

using fake::Plant;
using fake::PlantOptions;
using ackermann::State;
using ackermann::Controller;

// @TODO Daniel M. Sahu integrate noise throughout
// @TODO use proper (and consistent) common construction (build up)
// @TODO make targets random (with small range)
// @TODO control until success is achieved (or timeout)


/* @brief Test Fixture for repeated calls to a control loop. */
class AckermannControllerTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    // construct our base classes;
    opts_ = std::make_unique<PlantOptions>(1.0, 0.25, 0.2);

    // construct our plant
    plant_ = std::make_unique<Plant>(*opts_);

    // construct the controller
    controller_ = std::make_unique<Controller>();
  }

  // void TearDown() override {}

  // our base class members (we use pointers to avoid default construction)
  std::unique_ptr<PlantOptions> opts_;
  std::unique_ptr<Plant> plant_;
  std::unique_ptr<Controller> controller_;
};


/* @brief A convenience method for executing the given controller's command against the given Plant */
bool control_loop(std::unique_ptr<Plant>& p,
                  std::unique_ptr<Controller>& c,
                  double desired_speed,
                  double desired_heading,
                  double tolerance = 0.1,
                  double dt = 0.1,
                  int max_iterations = 10)
{
  int i = 0;
  while (i != max_iterations)
  {
    double current_speed, current_heading;
    p->getState(current_speed, current_heading);

    // calculate required command signal
    double w_l, w_r;
    c->update(current_speed, current_heading, desired_speed, desired_heading, w_l, w_r);

    // actually apply the command
    p->applyCommand(w_l, w_r, dt);

    // update time
    ++i;
  }

  // get final state
  double final_speed, final_heading;
  p->getState(final_speed, final_heading);

  if (abs(final_speed - desired_speed) > tolerance
      || abs(final_heading - desired_heading) > tolerance)
    return false;
  return true;
}

/* @brief Test that we've set up the Mock class properly. */
TEST_F(AckermannControllerTest, System_FakeSetup) {
  // try setting and getting the same state
  double speed_in = 1.3;
  double heading_in = 54.0;
  plant_->setState(speed_in, heading_in);

  double speed_out, heading_out;
  plant_->getState(speed_out, heading_out);
  EXPECT_EQ(speed_out, speed_in);
  EXPECT_EQ(heading_out, heading_in);

  // reset Plant and try setting a zero command (should result in zero state)
  plant_->setState(0.0, 0.0);
  plant_->applyCommand(0.0, 0.0, 0.0);
  plant_->getState(speed_out, heading_out);
  EXPECT_EQ(speed_out, 0.0);
  EXPECT_EQ(heading_out, 0.0);

  // make sure a dummy command returns an appropriate state
  plant_->setState(0.0, 0.0);
  plant_->applyCommand(0.5, 0.45, 0.1);
  plant_->getState(speed_out, heading_out);
  EXPECT_DOUBLE_EQ(speed_out, 0.59690260418206065);
  EXPECT_DOUBLE_EQ(heading_out, 0.032705133176410772);
}

/* @brief Test that the system converges to a desired setpoint w/ a zero noise Mock Plant. */
TEST_F(AckermannControllerTest, System_Convergence1) {
  EXPECT_TRUE(control_loop(plant_, controller_, 3.0, 45.0));
}

/* @brief Test that the system converges to a desired setpoint w/ a low noise Mock Plant. */
TEST_F(AckermannControllerTest, System_Convergence2) {
  EXPECT_TRUE(control_loop(plant_, controller_, 1.01, -45.0));
}

/* @brief Test that the system fails to converge to a "broken" Mock Plant. */
TEST_F(AckermannControllerTest, System_NoConvergence) {
  EXPECT_TRUE(control_loop(plant_, controller_, 100.0, 135.0));
}

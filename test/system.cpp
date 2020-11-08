/* @file system.cpp
 * @copyright [2020]
 */

#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <limits>
#include <Controller.hpp>
#include <Params.hpp>
#include "fake/plant.h"

#define WHEEL_BASE 1.5
#define TRACK_WIDTH 1.5
#define MAX_WHEEL_SEPARATION 45.0
#define CONTROL_FREQUENCY 100.0

using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::seconds;

/* @brief Test Fixture for repeated calls to a control loop. */
class AckermannControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // construct our base classes;

    // we only initialize our options / params classes once. This allows
    //  the user to modify their members and re-call 'SetUp'.
    if (!opts_)
      opts_ = std::make_unique<fake::PlantOptions>(WHEEL_BASE,
                                                   MAX_WHEEL_SEPARATION);
    if (!params_) {
      params_ = std::make_shared<ackermann::Params>(WHEEL_BASE, TRACK_WIDTH,
                                                    MAX_WHEEL_SEPARATION,
                                                    1.0,
                                                    1.0);
      // set some default control variables
      params_->control_frequency = CONTROL_FREQUENCY;
      params_->pid_speed->ki = 1.0;
      params_->pid_heading->ki = 1.0;
    }

    // construct our plant
    plant_ = std::make_unique<fake::Plant>(*opts_, params_);

    // construct the controller
    controller_ = std::make_unique<ackermann::Controller>(params_);

    // construct our limits class
    limits_ = std::make_unique<ackermann::Limits>(params_);
  }

  // our base class members (we use pointers to avoid default construction)
  std::unique_ptr<fake::Plant> plant_;
  std::unique_ptr<ackermann::Controller> controller_;
  std::unique_ptr<ackermann::Limits> limits_;

  // default initialize our params and options, so we can re-call SetUp
  std::shared_ptr<ackermann::Params> params_;
  std::unique_ptr<fake::PlantOptions> opts_;
};

/* @brief A convenience method for executing the given controller's
 * command against the given Plant.
 *
 */
bool control_loop(std::unique_ptr<fake::Plant>& p,
                  std::unique_ptr<ackermann::Controller>& c,
                  double desired_speed,
                  double desired_heading,
                  double max_duration,
                  double speed_tolerance = 0.1,
                  double heading_tolerance = 0.1) {
  double dt = 1.0/CONTROL_FREQUENCY;

  // update controller state from plant state
  double current_speed, current_heading;
  p->getState(current_speed, current_heading);

  // set goals and start control loop
  c->setState(current_speed, current_heading);
  c->setGoal(desired_speed, desired_heading);
  c->start();

  // initialize clock
  auto start = steady_clock::now();

  // loop until we've ran out of time
  while (duration_cast<seconds>(steady_clock::now() - start).count()
         < max_duration) {
    // calculate latest command
    double throttle, steering;
    c->getCommand(throttle, steering);

    // apply the command to our plant
    p->command(throttle, steering, dt);

    // get new state
    p->getState(current_speed, current_heading);

    // check whether or not we're within desired tolerance
    // (and can report success)
    if (std::abs(current_speed - desired_speed) < speed_tolerance
        || std::abs(current_heading - desired_heading) < heading_tolerance)
      return true;

    // check if we should break (stopped running, etc.)
    if (!c->isRunning())
      return false;
  }

  // if we've gotten this far, we've failed
  return false;
}

/* @brief Test that we've set up the Mock class properly. */
TEST_F(AckermannControllerTest, System_FakeSetup) {
  // try setting and getting the same state
  double speed_in = 1.3;
  double heading_in = 0.95;
  plant_->setState(speed_in, heading_in);

  double speed_out, heading_out;
  plant_->getState(speed_out, heading_out);
  EXPECT_DOUBLE_EQ(speed_out, speed_in);
  EXPECT_DOUBLE_EQ(heading_out, heading_in);

  // reset Plant and try setting a zero command (should result in zero state)
  plant_->setState(0.0, 0.0);
  plant_->command(0.0, 0.0, 0.0);
  plant_->getState(speed_out, heading_out);
  EXPECT_DOUBLE_EQ(speed_out, 0.0);
  EXPECT_DOUBLE_EQ(heading_out, 0.0);

  // make sure a dummy command returns an appropriate state
  plant_->setState(0.0, 0.0);
  plant_->command(0.5, 0.45, 0.1);
  plant_->getState(speed_out, heading_out);
  EXPECT_DOUBLE_EQ(limits_->speedToThrottle(speed_out), 0.5);
  EXPECT_NEAR(heading_out, 0.16, 0.01);
}

/* @brief Test that the system converges to a desired setpoint
 * w/ a zero noise Mock Plant.
 */
TEST_F(AckermannControllerTest, System_Convergence1) {
  EXPECT_TRUE(control_loop(plant_, controller_, 3.0, 1.2, 5.0));
}

/* @brief Test that the system converges to a desired setpoint
 * w/ a low noise Mock Plant.
 */
TEST_F(AckermannControllerTest, System_Convergence2) {
  // set noise and reset test fixture
  opts_->noise_mean = 0.0;
  opts_->noise_stddev = 0.05;
  SetUp();
  EXPECT_TRUE(control_loop(plant_, controller_, 1.01, -1.2, 5.0));
}

/* @brief Test that the system fails to converge to a "broken" Mock Plant. */
TEST_F(AckermannControllerTest, System_NoConvergence) {
  // set noise and reset test fixture
  opts_->noise_mean = 100.0;
  opts_->noise_stddev = 0.05;
  SetUp();
  EXPECT_FALSE(control_loop(plant_, controller_, 100.0, 135.0, 5.0));
}

#include <gtest/gtest.h>

#include <Controller.hpp>
#include "fake/plant.h"

using fake::Plant;
using fake::PlantOptions;
using ackermann::Controller;


// @TODO Daniel M. Sahu integrate noise throughout
// @TODO use proper (and consistent) common construction (build up)
// @TODO make targets random (with small range)
// @TODO control until success is achieved (or timeout)

/* @brief A convenience method for executing the given controller's command against the given Plant */
void control_loop(Plant& p,
                  Controller& c,
                  double desired_speed,
                  double desired_heading,
                  double dt,
                  double duration)
{
  double t = 0;
  while (t < duration)
  {
    double current_speed, current_heading;
    p.getState(current_speed, current_heading);

    // calculate required command signal
    double w_l, w_r;
    c.update(current_speed, current_heading, desired_speed, desired_heading, w_l, w_r);

    // actually apply the command
    p.applyCommand(w_l, w_r, dt);

    // update time
    t += dt;
  }
}

// construct fake plant
PlantOptions opts = PlantOptions(1.0, 0.25, 0.2);
Plant plant(opts);

// construct Controller and set appropriate parameters
Controller controller;

/* @brief Test that we've set up the Mock class properly. */
TEST(System_FakeSetup, should_pass) {
  // try setting and getting the same state
  double speed_in = 1.3;
  double heading_in = 54.0;

  plant.setState(speed_in, heading_in);

  double speed_out, heading_out;
  plant.getState(speed_out, heading_out);
  EXPECT_EQ(speed_out, speed_in);
  EXPECT_EQ(heading_out, heading_in);

  // reset Plant and try setting a zero command (should result in zero state)
  plant.setState(0.0, 0.0);
  plant.applyCommand(0.0, 0.0, 0.0);
  plant.getState(speed_out, heading_out);
  EXPECT_EQ(speed_out, 0.0);
  EXPECT_EQ(heading_out, 0.0);
}

/* @brief Test that the system converges to a desired setpoint w/ a zero noise Mock Plant. */
TEST(System_Convergence1, should_pass) {
  double desired_heading = 45.0;
  double desired_speed = 3.0;

  // reset plant
  plant.setState(0.0, 0.0);

  // perform control loop
  control_loop(plant, controller, desired_speed, desired_heading, 0.1, 10.0);

  // get final state
  double final_speed, final_heading;
  plant.getState(final_speed, final_heading);

  // check that we achieved our desired state
  EXPECT_EQ(final_speed, desired_speed);
  EXPECT_EQ(final_heading, desired_heading);
}

/* @brief Test that the system converges to a desired setpoint w/ a low noise Mock Plant. */
TEST(System_Convergence2, should_pass) {
  double desired_heading = -45.0;
  double desired_speed = 1.01;

  // reset plant
  plant.setState(0.0, 0.0);

  // perform control loop
  control_loop(plant, controller, desired_speed, desired_heading, 0.1, 10.0);

  // get final state
  double final_speed, final_heading;
  plant.getState(final_speed, final_heading);

  // check that we achieved our desired state
  EXPECT_EQ(final_speed, desired_speed);
  EXPECT_EQ(final_heading, desired_heading);
}

/* @brief Test that the system fails to converge to a "broken" Mock Plant. */
TEST(System_NoConvergence, should_pass) {
  double desired_heading = 135.0;
  double desired_speed = 100.0;

  // reset plant
  plant.setState(0.0, 0.0);

  // perform control loop
  control_loop(plant, controller, desired_speed, desired_heading, 0.1, 10.0);

  // get final state
  double final_speed, final_heading;
  plant.getState(final_speed, final_heading);

  // check that we achieved our desired state
  EXPECT_EQ(final_speed, desired_speed);
  EXPECT_EQ(final_heading, desired_heading);
}

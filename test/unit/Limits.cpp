/* @file Limits.cpp
 * @copyright [2020]
 */

#include <gtest/gtest.h>
#include <Limits.hpp>
#include <Params.hpp>
#include <cmath>

using ackermann::Limits;
using ackermann::Params;

/* @brief Test limiting without any limits (should be default behavior).*/
TEST(Limits_NoLimit, should_pass) {
  double dt = 0.01;
  auto p = std::make_shared<Params>(0.0, 1e8, 0.0, 0.0);
  Limits lim(p);

  {
    // check #1: we don't limit 0 moves
    double original_throttle = 0.0;
    double original_steering = 0.0;
    double desired_throttle = original_throttle;
    double desired_steering = original_steering;
    double current_steering_vel = 0;
    double desired_steering_vel = current_steering_vel;
    lim.limit(original_throttle, original_steering, current_steering_vel,
      desired_throttle, desired_steering, desired_steering_vel, dt);
    EXPECT_DOUBLE_EQ(original_throttle, desired_throttle);
    EXPECT_DOUBLE_EQ(original_steering, desired_steering);
  }

  {
    // check #2: we shouldn't limit arbitrary values
    double original_throttle = .5;
    double original_steering = 1000;
    double desired_throttle = original_throttle;
    double desired_steering = original_steering;
    double current_steering_vel = 0;
    double desired_steering_vel = current_steering_vel;
    lim.limit(original_throttle, original_steering, current_steering_vel,
      desired_throttle, desired_steering, desired_steering_vel, dt);
    EXPECT_DOUBLE_EQ(original_throttle, desired_throttle);
    EXPECT_DOUBLE_EQ(original_steering, desired_steering);
  }

  {
    // check #3: make sure negative steering works properly too
    // no negative throttle values allowed
    double original_throttle = .5;
    double original_steering = -1000000;
    double desired_throttle = original_throttle;
    double desired_steering = original_steering;
    double current_steering_vel = 0;
    double desired_steering_vel = current_steering_vel;
    lim.limit(original_throttle, original_steering, current_steering_vel,
      desired_throttle, desired_steering, desired_steering_vel, dt);
    EXPECT_DOUBLE_EQ(original_throttle, desired_throttle);
    EXPECT_DOUBLE_EQ(original_steering, desired_steering);
  }
}

/* @brief Test velocity limiting.*/
TEST(Limits_Velocity, should_pass) {
  // instantiate a limits class with only velocity limits
  double dt = 0.01;
  auto p = std::make_shared<Params>(0.0, 0.0, 0.0, 0.0);
  p->velocity_max = 1.0;
  Limits lim(p);

  {
    // check #1: make sure we throttle velocity (only!)
    double original_throttle = 1000000;
    double original_steering = 0;
    double desired_throttle = original_throttle;
    double desired_steering = original_steering;
    double current_steering_vel = 0;
    double desired_steering_vel = current_steering_vel;
    lim.limit(original_throttle, original_steering, current_steering_vel,
      desired_throttle, desired_steering, desired_steering_vel, dt);
    EXPECT_DOUBLE_EQ(desired_throttle, lim.speedToThrottle(p->velocity_max));
    EXPECT_DOUBLE_EQ(original_steering, desired_steering);
  }

  {
    // check #2: make sure negatives floor properly
    double original_throttle = -1000000;
    double original_steering = 0;
    double desired_throttle = original_throttle;
    double desired_steering = original_steering;
    double current_steering_vel = 0;
    double desired_steering_vel = current_steering_vel;
    lim.limit(original_throttle, original_steering, current_steering_vel,
      desired_throttle, desired_steering, desired_steering_vel, dt);
    EXPECT_DOUBLE_EQ(desired_throttle, lim.speedToThrottle(p->velocity_min));
    EXPECT_DOUBLE_EQ(original_steering, desired_steering);
  }
}

/* @brief Test acceleration limiting.*/
TEST(Limits_Acceleration, should_pass) {
  // instantiate a limits class with only acceleration limits
  double dt = 0.01;
  auto p = std::make_shared<Params>(0.0, 0.0, 0.0, 0.0);
  p->velocity_max = 100.0;
  p->velocity_min = 0.0;
  p->acceleration_min = -0.001;
  p->acceleration_max = 0.001;
  Limits lim(p);

  {
    double original_throttle = 0;
    double original_steering = 0;
    double desired_throttle = 1;
    double desired_steering = original_steering;
    double current_steering_vel = 0;
    double desired_steering_vel = current_steering_vel;
    lim.limit(0.0, 0.0, 0.0, desired_throttle, desired_steering, desired_steering_vel, dt);
    EXPECT_DOUBLE_EQ(lim.throttleToSpeed(desired_throttle), (p->acceleration_max*dt));
    EXPECT_DOUBLE_EQ(original_steering, desired_steering);
  }

  {
    // check #2: make sure negatives work properly too
    double original_throttle = 0;
    double original_steering = 0;
    double desired_throttle = -1;
    double desired_steering = original_steering;
    double current_steering_vel = 0;
    double desired_steering_vel = current_steering_vel;
    lim.limit(0.0, 0.0, 0.0, desired_throttle, desired_steering, desired_steering_vel, dt);
    EXPECT_DOUBLE_EQ(lim.throttleToSpeed(desired_throttle), 0);
    EXPECT_DOUBLE_EQ(original_steering, desired_steering);
  }
}


/* @brief Test, heading limitation (max angle)*/
TEST(Limits_Heading_Angle, should_pass) {
  // instantiate a limits class with only heading limit
  double dt = 0.01;
  auto p = std::make_shared<Params>(0.0, 1.0, 0.0, 0.0);
  Limits lim(p);

  {
    double original_steering = 0;
    double desired_steering = 2;
    double current_steering_vel = 0;
    double desired_steering_vel = current_steering_vel;
    double desired_throttle = 1;
    lim.limit(1, original_steering, current_steering_vel, desired_throttle, desired_steering, desired_steering_vel, dt);
    EXPECT_DOUBLE_EQ(desired_steering, p->max_steering_angle);
  }

  {
    // check #2: make sure negatives work properly too
    double original_steering = 0;
    double desired_steering = -2;
    double current_steering_vel = 0;
    double desired_steering_vel = current_steering_vel;
    double desired_throttle = 1;
    lim.limit(1, original_steering, current_steering_vel, desired_throttle, desired_steering, desired_steering_vel, dt);
    EXPECT_DOUBLE_EQ(desired_steering, (-1*p->max_steering_angle));
  }
}

/* @brief Test, heading limitation (max angular velocity)*/
TEST(Limits_Heading_AngularVelocity, should_pass) {
  // instantiate a limits class with only heading limit
  double dt = 0.01;
  auto p = std::make_shared<Params>(0.0, 3.0, 0.0, 0.0);
  p->angular_velocity_max = 0.1;
  p->angular_velocity_min = -0.1;
  Limits lim(p);

  {
    double original_steering = 0;
    double desired_steering = 2;
    double current_steering_vel = 0;
    double desired_steering_vel = current_steering_vel;
    double desired_throttle = 1;
    lim.limit(1, original_steering, current_steering_vel, desired_throttle, desired_steering, desired_steering_vel, dt);
    EXPECT_DOUBLE_EQ(desired_steering_vel, p->angular_velocity_max);
    EXPECT_DOUBLE_EQ(desired_steering, desired_steering_vel*dt);
  }

  {
    // check #2: make sure negatives work properly too
    double original_steering = 0;
    double desired_steering = -2;
    double current_steering_vel = 0;
    double desired_steering_vel = current_steering_vel;
    double desired_throttle = 1;
    lim.limit(1, original_steering, current_steering_vel, desired_throttle, desired_steering, desired_steering_vel, dt);
    EXPECT_DOUBLE_EQ(desired_steering_vel, p->angular_velocity_min);
    EXPECT_DOUBLE_EQ(desired_steering, desired_steering_vel*dt);
  }
}

/* @brief Test, heading limitation (max angular acceleration)*/
TEST(Limits_Heading_AngularAcceleration, should_pass) {
  // instantiate a limits class with only heading limit
  double dt = 0.01;
  auto p = std::make_shared<Params>(0.0, 1.0, 0.0, 0.0);
  p->angular_acceleration_max = 0.01;
  p->angular_acceleration_min = -0.01;
  Limits lim(p);

  {
    double original_steering = 0;
    double desired_steering = 2;
    double current_steering_vel = 0;
    double desired_steering_vel = current_steering_vel;
    double current_throttle = 1.0;
    double desired_throttle = current_throttle;
    lim.limit(current_throttle, original_steering, current_steering_vel, desired_throttle, desired_steering, desired_steering_vel, dt);
    EXPECT_DOUBLE_EQ((desired_steering_vel - current_steering_vel)/dt, p->angular_acceleration_max);
    EXPECT_DOUBLE_EQ(desired_steering_vel, p->angular_acceleration_max*dt);
    EXPECT_DOUBLE_EQ(desired_steering, desired_steering_vel*dt);
  }

  {
    // check #2: make sure negatives work properly too
    double original_steering = 0;
    double desired_steering = -2;
    double current_steering_vel = 0;
    double desired_steering_vel = current_steering_vel;
    double current_throttle = 1.0;
    double desired_throttle = current_throttle;
    lim.limit(current_throttle, original_steering, current_steering_vel, desired_throttle, desired_steering, desired_steering_vel, dt);
    EXPECT_DOUBLE_EQ((desired_steering_vel - current_steering_vel)/dt, p->angular_acceleration_min);
    EXPECT_DOUBLE_EQ(desired_steering_vel, p->angular_acceleration_min*dt);
    EXPECT_DOUBLE_EQ(desired_steering, desired_steering_vel*dt);
  }
}

TEST(Throttle_to_Speed_Conversion, should_pass) {
  // instantiate a limits class with only velocity limits
  double dt = 0.01;
  auto p = std::make_shared<Params>(0.0, 0.0, 0.0, 0.0);
  p->velocity_max = 100.0;
  Limits lim(p);

  // chceck #1: make sure a value in range works correctly
  {
    double throttle = 0.1;
    double calc_speed = lim.throttleToSpeed(throttle);
    EXPECT_DOUBLE_EQ(calc_speed,throttle*p->velocity_max);
  }
  // check #2: make sure to ceiling high values
  {
    double throttle = 1.1;
    double calc_speed = lim.throttleToSpeed(throttle);
    EXPECT_DOUBLE_EQ(calc_speed,p->velocity_max);
  }
  // check #3: make sure to floor low values
  {
    double throttle = -0.5;
    double calc_speed = lim.throttleToSpeed(throttle);
    EXPECT_DOUBLE_EQ(calc_speed,0.0);
  }
}

TEST(Speed_to_Throttle_Conversion, should_pass) {
  // instantiate a limits class with only velocity limits
  double dt = 0.01;
  auto p = std::make_shared<Params>(0.0, 0.0, 0.0, 0.0);
  p->velocity_max = 100.0;
  Limits lim(p);

  // chceck #1: make sure a value in range works correctly
  {
    double speed = 10.0;
    double calc_throttle = lim.speedToThrottle(speed);
    EXPECT_DOUBLE_EQ(calc_throttle,speed/p->velocity_max);
  }
  // check #2: make sure to ceiling high values
  {
    double speed = 200.0;
    double calc_throttle = lim.speedToThrottle(speed);
    EXPECT_DOUBLE_EQ(calc_throttle,p->throttle_max);
  }
  // check #3: make sure to floor low values
  {
    double speed = -10.0;
    double calc_throttle = lim.speedToThrottle(speed);
    EXPECT_DOUBLE_EQ(calc_throttle,p->throttle_min);
  }
}

TEST(HeadingLimits, should_pass) {
  auto p = std::make_shared<Params>(0.0, 1e8, 0.0, 0.0);
  Limits lim(p);
  // check 1 - within bound
  {
    double desired_heading = M_PI/2;
    double calc_heading = lim.boundHeading(desired_heading);
    EXPECT_DOUBLE_EQ(desired_heading,calc_heading);
  }
  // check 2 - negative
  {
    double desired_heading = -M_PI/2;
    double calc_heading = lim.boundHeading(desired_heading);
    EXPECT_DOUBLE_EQ(3*M_PI/2,calc_heading);
  }
  // check 3 - positive
  {
    double desired_heading = 5*M_PI/2;
    double calc_heading = lim.boundHeading(desired_heading);
    EXPECT_DOUBLE_EQ(M_PI/2,calc_heading);
  }
}

TEST(ShortestArcToTurn, should_pass) {
  auto p = std::make_shared<Params>(0.0, 1e8, 0.0, 0.0);
  Limits lim(p);
  // check 1 - right turn
  {
    double current_heading = M_PI/2;
    double desired_heading = (3*M_PI/2)-0.1;
    double arc = lim.shortestArcToTurn(current_heading, desired_heading);
    EXPECT_DOUBLE_EQ(arc,(M_PI-0.1));
  }
  // check 2 - left turn
  {
    double current_heading = M_PI/2;
    double desired_heading = (3*M_PI/2)+0.1;
    double arc = lim.shortestArcToTurn(current_heading, desired_heading);
    EXPECT_DOUBLE_EQ(arc,-(M_PI-0.1));
  }

}

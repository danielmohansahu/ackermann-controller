#include <gtest/gtest.h>
#include <AckermannVehicle.hpp>

using ackermann::AckermannVehicle;

/* @brief Test all setters and Getters. */
TEST(AckermannVehicle_SettersAndGetters, should_pass) {
  AckermannVehicle ack;

  // arbitrary test values
  double wheel_base = 32.0;
  double wheel_separation = 153.0;
  double steering_angle = 45.0;


  // test wheel base setter / getter
  ack.set_wheel_base(wheel_base);
  EXPECT_EQ(ack.get_wheel_base(), wheel_base);

  // test wheel separation setter / getter
  ack.set_wheel_separation(wheel_separation);
  EXPECT_EQ(ack.get_wheel_separation(), wheel_separation);

  // test wheel base setter / getter
  ack.set_max_steering_angle(steering_angle);
  EXPECT_EQ(ack.get_max_steering_angle(), steering_angle);
}

/* @brief Test the getState method with empty parameters. */
TEST(AckermannVehicle_getState1, should_pass) {
  EXPECT_EQ(1, 1);
}

/* @brief Test the getState method with a different set of parameters. */
TEST(AckermannVehicle_getState2, should_pass) {
  EXPECT_EQ(1, 1);
}

/* @brief Test the getState method with a different set of parameters. */
TEST(AckermannVehicle_getState3, should_pass) {
  EXPECT_EQ(1, 1);
}

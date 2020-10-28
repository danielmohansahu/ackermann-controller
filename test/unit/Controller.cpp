/* @file Controller.cpp
 * @copyright [2020]
 */

#include <gtest/gtest.h>
#include <Controller.hpp>
#include <Params.hpp>

/* @brief Test Fixture for repeated independent construction of the Controller. */
class AckemannControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // construct our base classes;
    params_ = std::make_shared<ackermann::Params>(1.25, 45.0, 1.01, 0.15);
    controller_ = std::make_unique<ackermann::Controller>(params_);
  }

  // our base class members (we use pointers to avoid default construction)
  std::unique_ptr<ackermann::Controller> controller_;
  std::shared_ptr<ackermann::Params> params_;
};

/* @brief Test Controller with default params */
TEST_F(AckemannControllerTest, ControllerSettingAndGetting) {
  // test setting and getting state
  {
    double speed, heading, speed_out, heading_out;

    // default state should be 0.0
    controller_->getState(speed, heading);
    EXPECT_EQ(speed, 0.0);
    EXPECT_EQ(heading, 0.0);

    // set to arbitrary value and make sure it actually happens
    speed = 2.0;
    heading = -21.0;
    controller_->setState(speed, heading);
    controller_->getState(speed_out, heading_out);
    EXPECT_EQ(speed, speed_out);
    EXPECT_EQ(heading, heading_out);
  }

  // test setting and getting goal
  {
    double speed, heading, speed_out, heading_out;

    // default goal should be 0.0
    controller_->getGoal(speed, heading);
    EXPECT_EQ(speed, 0.0);
    EXPECT_EQ(heading, 0.0);

    // set to arbitrary value and make sure it actually happens
    speed = 2.0;
    heading = -21.0;
    controller_->setGoal(speed, heading);
    controller_->getGoal(speed_out, heading_out);
    EXPECT_EQ(speed, speed_out);
    EXPECT_EQ(heading, heading_out);
  }
}

/* @brief Test Controller with default params */
// @TODO Daniel M. Sahu: Expand this test. It's a little too simple right now.
TEST_F(AckemannControllerTest, ControllerThreading) {

  // try a simple start / stop
  {
    controller_->start();
    EXPECT_TRUE(controller_->isRunning());
    controller_->stop();
    EXPECT_FALSE(controller_->isRunning());
  }

  // test multiple calls
  {
    controller_->start();
    EXPECT_TRUE(controller_->isRunning());
    controller_->start();
    EXPECT_TRUE(controller_->isRunning());
    controller_->start();
    EXPECT_TRUE(controller_->isRunning());
    controller_->stop();
    EXPECT_FALSE(controller_->isRunning());
    controller_->stop();
    EXPECT_FALSE(controller_->isRunning());
    controller_->stop();
    EXPECT_FALSE(controller_->isRunning());
  }

  // make sure reset and other async calls don't break anything
  {
    controller_->start();
    EXPECT_TRUE(controller_->isRunning());

    // really just testing for corrupted data
    double speed, heading, throttle, steering;
    for (unsigned int i = 0; i != 10000; ++i)
    {
      // call some random accessors
      controller_->setState(1.0, -15.0);
      controller_->getState(speed, heading);

      controller_->setGoal(-1.0, 15.0);
      controller_->getGoal(throttle, steering);

      if (!controller_->isRunning()
          || (speed != 1.0)
          || (heading != -15.0)
          || (throttle != -1.0)
          || (steering != 15.0))
      {
        std::cerr << "Encountered error in control loop; probably a threading issue." << std::endl;
        EXPECT_FALSE(true);
        break;
      }
      // reset to original state (0.0 for everything, hopefully)
      controller_->reset();
    }      

    controller_->stop();
    EXPECT_FALSE(controller_->isRunning());
  }
}

/* @brief Test Controller with default params */
// @TODO Daniel M. Sahu: Write this test.
TEST_F(AckemannControllerTest, ControllerTiming) {
  // 
}

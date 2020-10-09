/* @brief A Fake implementation of a physical Ackermann platform.
 */

#include <iostream>
#include <cmath>
#include <random>

namespace fake
{

/* @brief A Fake Plant for use in testing our Controller.
 * 
 * This class just maintains some simple state information and
 * applies a small amount of noise (if desired).
 */
class Plant
{
 public:
  Plant(double wheel_base, double wheel_separation, double wheel_radius,
        double max_steering_angle, double noise_mean = 0.0, double noise_stddev = 0.0)
  : wheel_base_(wheel_base), 
    wheel_separation_(wheel_separation),
    wheel_radius_(wheel_radius),
    max_steering_angle_(max_steering_angle),
    noise_mean_(noise_mean),
    noise_stddev_(noise_stddev),
    dist_(noise_mean, noise_stddev)
  {
  }

  /* @brief Set the current system state */
  void setState(double speed, double heading);

  /* @brief Get the current system state */
  void getState(double& speed, double& heading);

  /* @brief Simulate a command */
  void applyCommand(double w_left, double w_right, double dt);

 private:
  // ackermann parameters
  double wheel_base_ {0.0};
  double wheel_separation_ {0.0};
  double wheel_radius_ {0.0};
  double max_steering_angle_ {0.0};

  // noise variables
  double noise_mean_ {0.0};
  double noise_stddev_ {0.0};

  // state variables
  double heading_;
  double speed_;

  // random noise generation
  std::default_random_engine generator_;
  std::normal_distribution<double> dist_;
};

} // namespace fake
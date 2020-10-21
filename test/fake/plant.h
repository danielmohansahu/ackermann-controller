#pragma once
/* @brief A Fake implementation of a physical Ackermann platform (bicycle geometry).
 * 
 * @author Daniel M.
 * 
 * @copyright [2020]
 */

#include <assert.h>
#include <iostream>
#include <limits>
#include <cmath>
#include <random>

namespace fake {

/* @brief Configurable parameters for the Plant class.
 */
struct PlantOptions {
  // ackermann parameters
  double wheel_base;
  double max_steering_angle;

  // noise model parameters
  double noise_mean {0.0};
  double noise_stddev {0.0};

  /* Delete default constructor */
  PlantOptions() = delete;

  /* Constructor for required components */
  PlantOptions(double wheel_base_, double max_steering_angle_)
    : wheel_base(wheel_base_),
      max_steering_angle(max_steering_angle_) {
  }

  /* @brief Print out the current set of state variables */
  void print() {
    std::cout << "PlantOptions: " << std::endl;
    std::cout << "\twheel_base: " << wheel_base << std::endl;
    std::cout << "\tmax_steering_angle: " << max_steering_angle << std::endl;
    std::cout << "\tnoise_mean: " << noise_mean << std::endl;
    std::cout << "\tnoise_stddev: " << noise_stddev << std::endl;
  }
};

/* @brief A Fake Plant for use in testing our Controller.
 * 
 * This class just maintains some simple state information and
 * applies a small amount of noise (if desired).
 */
class Plant {
 public:
  explicit Plant(const PlantOptions& opts)
  : speed_(0.0),
    heading_(0.0),
    opts_(opts),
    dist_(opts.noise_mean, opts.noise_stddev) {
  }

  /* @brief Reset all state variables to their defaults. */
  void reset();

  /* @brief Set the current system state.
   *
   * @param speed: The new system speed.
   * @param heading: The new system heading.
   */
  void setState(const double speed, const double heading);

  /* @brief Get the current system state.
   * 
   * @param speed: The current system speed.
   * @param heading: The current system heading.
   */
  void getState(double& speed, double& heading) const;

  /* @brief Simulate an actual command to the vehicle.
   * 
   * This is a very simple approximation of the plant; the throttle
   * command is assumed to translate instantly into the new speed,
   * and the new heading is calculated by integrating the effect of
   * the new steering angle over the timestep.
   * 
   * @param throttle: The throttle command to apply.
   * @param steering: The steering command to apply.
   * @param dt: The time duration of the command.
   */
  void command(const double throttle, const double steering, const double dt);

 private:
  // state variables
  double speed_;
  double heading_;

  // struct of our system options
  PlantOptions opts_;

  // random noise generation
  std::default_random_engine generator_;
  std::normal_distribution<double> dist_;
};

} // namespace fake

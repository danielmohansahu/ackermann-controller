#pragma once
/**
 * @file plant.h
 * @brief A Fake implementation of a physical Ackermann platform.
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
#include <atomic>
#include <memory>
#include <thread>
#include <chrono>
#include "Params.hpp"
#include "Limits.hpp"

/**
* @brief Namespace for fake plant model implementation
*/
namespace fake {

 /**
 * @brief Configurable parameters for the Plant class.
 */
struct PlantOptions {
  // ackermann parameters
  /**
  * @brief Length between front and rear axles (m)
  */
  double wheel_base;
  /**
  * @brief Maximum steering angle (rad)
  */
  double max_steering_angle;

  // noise model parameters
  /**
  * @brief Mean noise parameter for noise modeling.
  */
  double noise_mean {0.0};
  /**
  * @brief StdDev noise parameter for noise modeling.
  */
  double noise_stddev {0.0};

  /* Delete default constructor */
  PlantOptions() = delete;

  /* Constructor for required components */
  /**
  * @brief Constructor for PlantOptions structure
  * @param wheel_base Distance between front and rear axles (m)
  * @param max_steering_angle Maximum steering angle (rad)
  */
  PlantOptions(double wheel_base_,
              double max_steering_angle_)
    : wheel_base(wheel_base_),
      max_steering_angle(max_steering_angle_) {
  }

  /**
  * @brief Print out the current set of state variables
  */
  void print() {
    std::cout << "PlantOptions: " << std::endl;
    std::cout << "\twheel_base: " << wheel_base << std::endl;
    std::cout << "\tmax_steering_angle: " << max_steering_angle << std::endl;
    std::cout << "\tnoise_mean: " << noise_mean << std::endl;
    std::cout << "\tnoise_stddev: " << noise_stddev << std::endl;
  }
};

/**
* @brief A Fake Plant for use in testing our Controller.
 *
 * This class just maintains some simple state information and
 * applies a small amount of noise (if desired).
 */
class Plant {
 public:
   /**
   * @brief Constructor
    *
    * @param opts Plant specifications (wheel base, max steering)
    * @param params Shared pointer; model parameters.
    */
  explicit Plant(const PlantOptions& opts,
     const std::shared_ptr<const ackermann::Params>& params);

  /**
  * @brief Reset all state variables to their defaults.
  */
  void reset();

  /**
  * @brief Set the current system state.
   *
   * @param speed: The new system speed.
   * @param heading: The new system heading.
   */
  void setState(const double speed, const double heading);

  /**
  * @brief Get the current system state.
   *
   * @param speed: The current system speed.
   * @param heading: The current system heading.
   */
  void getState(double& speed, double& heading) const;

  /**
  * @brief Simulate an actual command to the vehicle.
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
   /**
   * @brief speed variable
   */
  double speed_;
  /**
  * @brief heading variable
  */
  double heading_;

  /**
  * @brief struct of our system options
  */
  PlantOptions opts_;

  /**
  * @brief A copy of our configuration parameters.
  */
  std::shared_ptr<const ackermann::Params> params_;

  /**
  * @brief random noise generation
  */
  std::default_random_engine generator_;
  std::normal_distribution<double> dist_;

  /**
  * @brief Object used to apply kinematic constraints to
   * a calculated command (to prevent saturation)
   */
  std::unique_ptr<ackermann::Limits> limits_;
};

}  // namespace fake

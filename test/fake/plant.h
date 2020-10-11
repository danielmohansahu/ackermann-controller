/* @brief A Fake implementation of a physical Ackermann platform.
 */

#include <iostream>
#include <cmath>
#include <random>
#include <limits>

namespace fake
{

/* @brief Configurable parameters for the Plant class.
 */
struct PlantOptions
{
  // ackermann parameters
  double wheel_base;
  double wheel_separation;
  double wheel_radius;
  double max_steering_angle {std::numeric_limits<double>::max()};

  // noise model parameters
  double noise_mean {0.0};
  double noise_stddev {0.0};

  /* Delete default constructor */
  PlantOptions() = delete;

  /* Constructor for required components */
  PlantOptions(double wheel_base_, double wheel_separation_, double wheel_radius_)
   : wheel_base(wheel_base_),
     wheel_separation(wheel_separation_),
     wheel_radius(wheel_radius_)
  {
  }

  /* @brief Print out the current set of state variables */
  void print()
  {
    std::cout << "PlantOptions: " << std::endl;
    std::cout << "\twheel_base: " << wheel_base << std::endl;
    std::cout << "\twheel_separation: " << wheel_separation << std::endl;
    std::cout << "\twheel_radius: " << wheel_radius << std::endl;
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
class Plant
{
 public:
  Plant(const PlantOptions& opts)
  : speed_(0.0),
    heading_(0.0),
    opts_(opts),
    dist_(opts.noise_mean, opts.noise_stddev)
  {
  }

  /* @brief Set the current system state */
  void setState(double speed, double heading);

  /* @brief Get the current system state */
  void getState(double& speed, double& heading);

  /* @brief Simulate a command */
  void applyCommand(double w_left, double w_right, double dt);

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
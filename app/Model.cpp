/* @file Model.cpp
 * @brief This is the class definition for a vehicle which
 * uses an Ackermann Steering Controller.
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 *
 * @copyright [2020]
 * 
 * http://www.iaeng.org/publication/WCE2016/WCE2016_pp1062-1066.pdf
 * https://www.xarg.org/book/kinematics/ackerman-steering/
 */

#include <Model.hpp>

namespace ackermann {

Model::Model(const double wheel_base, const double max_steering_angle)
			: wheel_base_(wheel_base),
			  max_steering_angle_(max_steering_angle)
			{}

void Model::reset() {
desired_speed_ = 0.0;
desired_heading_ = 0.0;
current_speed_ = 0.0;
current_heading_ = 0.0;
current_throttle_ = 0.0;
current_steering_ = 0.0;
}

void Model::setState(const double speed, const double heading) {
	current_speed_ = speed;
	current_heading_ = heading;

}

void Model::getState(double& speed, double& heading) const {
	speed = current_speed_;
	heading = current_heading_;
}

void Model::setGoal(const double speed, const double heading) {
	desired_speed_ = speed;
	desired_heading_ = heading;
}

void Model::getGoal(double& speed, double& heading) const {
	speed = desired_speed_;
	heading = desired_heading_;
}

void Model::getCommand(double& throttle, double& steering) const {
	throttle = current_throttle_;
	steering = current_steering_;
}

void Model::command(const double throttle, const double steering, const double dt) const {
	current_throttle_ = throttle;
	current_steering_ = steering;
	
	//TODO: Update the current speed and heading based on dt and wheelbase and steering angle
}

void Model::getError(double& speed_error, double& heading_error) const {
	speed_error = desired_speed_ - current_speed_;
	heading_error = desired_heading_ - current_heading_;
}

} // namespace ackermann

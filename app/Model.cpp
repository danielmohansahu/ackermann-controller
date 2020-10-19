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

Model::Model(const double wheel_base, const double max_steering_angle) {

}

void Model::reset() {

}

void Model::setState(const double speed, const double heading) {

}

void Model::getState(double& speed, double& heading) const {

}

void Model::setGoal(const double speed, const double heading) {

}

void Model::getGoal(double& speed, double& heading) const {

}

void Model::getCommand(double& throttle, double& steering) const {

}

void Model::command(const double throttle, const double steering, const double dt) const {

}

void Model::getError(double& speed_error, double& heading_error) const {

}

} // namespace ackermann

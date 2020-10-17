/* @file Controller.cpp
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Sentosh Kesani
 *
 * @copyright [2020]
 */

// @TODO Currently STUB implementation; needs to be filled

#include <Controller.hpp>

namespace ackermann {

Controller::Controller(const Params& params) {
}

void Controller::start() {
}

void Controller::stop() {
}

void Controller::reset() {
}

void Controller::setState(const double heading, const double speed) {
}

void Controller::getState(double& heading, double& speed) const {
}

void Controller::setGoal(const double heading, const double speed) {
}

void Controller::getGoal(double& heading, double& speed) const {
}

void Controller::getCommand(double& throttle, double& steering) const {
}

void Controller::controlLoop() {
}

} // namespace ackermann

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
	this->model_ = std::make_unique<Model>(new Model(params.wheel_base, params.max_steering_angle));
	this->limits_ = std::make_unique<Limits>(new Limits(params));
	this->pid_speed_ = std::make_unique<PID>(new PID(params.kp_speed, params.ki_speed, params.kd_speed));
	this->pid_heading_ = std::make_unique<PID>(new PID(params.kp_heading, params.ki_heading, params.kd_heading));
}

void Controller::start() {
	this->running_ = true;
	control_loop_handle_(controlLoop);
	control_loop_handle_.join();	
}

void Controller::stop() {
	this->running_ = false;
	this->reset();
}

void Controller::reset() {
	this->model_.reset(nullptr);
	this->limits_.reset(nullptr);
	this->pid_speed_.reset(nullptr);
	this->pid_heading_.reset(nullptr);
}

bool Controller::isRunning() const {
	return running_;
}

void Controller::setState(const double heading, const double speed) {
	this->model_->setState(heading, speed);
}

void Controller::getState(double& heading, double& speed) const {
	this->model_->getState(heading, speed);
}

void Controller::setGoal(const double heading, const double speed) {
	this->model_->setGoal(heading, speed);
}

void Controller::getGoal(double& heading, double& speed) const {
	this->model_->getGoal(heading, speed);
}

void Controller::getCommand(double& throttle, double& steering) const {
	this->model_->getCommand(heading, speed);
}

void Controller::controlLoop() {
	//TODO: A while loop for the controller
	while(running_) {
		// TODO: Use PIDs, Limits and Model pointers to evaluate.
	}
}

} // namespace ackermann

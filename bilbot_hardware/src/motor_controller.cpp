#include "bilbot_hardware/motor_controller.hpp"
#include <pigpiod_if2.h>

namespace bilbot_hardware {

motor_controller::motor_controller(int pi, int side, int gpioA, int gpioB, float kp, float kd, float ki) 
	: kp_(kp), kd_(kd), ki_(ki), side_(side)
{
	set_mode(pi, gpioA, PI_OUTPUT);
	set_mode(pi, gpioB, PI_OUTPUT);

	set_pull_up_down(pi, gpioA, PI_PUD_DOWN);
	set_pull_up_down(pi, gpioB, PI_PUD_DOWN);

	set_PWM_frequency(pi, gpioA, 40000);
	set_PWM_frequency(pi, gpioB, 40000);
}

motor_controller::~motor_controller() {

}

void motor_controller::set_error() {
	error_ = wheel_cmd_ - wheel_vel_;
	error_old_ = error_;
}

void motor_controller::set_wheel_cmd(float wheel_cmd) {
	wheel_cmd_ = wheel_cmd;
}

void motor_controller::set_wheel_vel(float wheel_vel) {
	wheel_vel_ = wheel_vel;
}

void motor_controller::filter_velocity() {
	err_d_ = (error_ - error_old_)/0.01;
	err_d_ = (err_d_ + err_d_old1_ + err_d_old2_)/3;

	err_d_old2_ = err_d_old1_;
	err_d_old1_ = err_d_;

}

void motor_controller::estimate_integral() {
	err_i_ = err_i_old_ + (error_ + error_old_)/2*0.01;
	err_i_old_ = err_i_;

}

float motor_controller::control() {
	float u = kp_*error_ + kd_*err_d_;
	if (fabs(error_ < integral_threshold)) {
		estimate_integral();
		u = u +ki_*err_i_;
	} else {
		err_i_ = 0;
		err_i_old_ = 0;
	}

	if (u >= 255) {
		u = 255;
		err_i_ = err_i_old_;
	} else if (u < -255) {
		u = -255;
		err_i_ = err_i_old_;
	}

	return u; 
}

void motor_controller::commandCallback(const bilbot_msgs::Drive::ConstPtr& cmd_vel) {
	set_wheel_cmd(cmd_vel->drivers[side_]);
}

void motor_controller::stateCallback(const sensor_msgs::JointState::ConstPtr& curr_vel) {
	set_wheel_vel(wheel_vel_ = curr_vel->velocity[side_]);
}

}
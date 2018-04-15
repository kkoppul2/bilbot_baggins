#include "bilbot_hardware/motor_controller.hpp"

namespace bilbot_hardware {

motor_controller::motor_controller(int gpioA, int gpioB, float kp, float kd, float ki) 
	: kp_(kp), kd_(kd), ki_(ki)
{
	gpioSetMode(gpioA, PI_OUTPUT);
	gpioSetMode(gpioA, PI_OUTPUT);

	gpioSetPUllUpDown(gpioA, PI_PUD_DOWN);
	gpioSetPUllUpDown(gpioB, PI_PUD_DOWN);

	gpioSetPWMFrequency(gpioA, 50000);
	gpioSetPWMFrequency(gpioB, 50000);
}

motor_controller::~motor_controller() {

}

void motor_controller::set_error(float err) {
	error_ = err;
	error_old_ = error_;
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

void motor_controller::commandCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
	wheel_cmd = cmd_vel->angular.z;
}

void motor_controller::stateCallback(const sensor_msgs::JointState::ConstPtr& curr_vel) {
	wheel_vel = curr_vel-velocity;
}

}
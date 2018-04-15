#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "math.h"
#include <pigpio.h>
#include "sensor_msgs/JointState.h"
#include "bilbot_msgs/Drive.h"
#include "sensor_msgs/JointState.h"

namespace bilbot_hardware {

class motor_controller
{
private:
	float kp_, kd_, ki_; //controller gains

	//Error
	float error_, error_old_;

	//Derivative Variables
	float err_d_, err_d_old1_, err_d_old2_;

	//Integral Variables
	float err_i_, err_i_old_;
	float integral_threshold;

	//Gpio pins
	int gpioA, gpioB;

	int side_;

	void filter_velocity();
	void estimate_integral();

	float wheel_cmd_, wheel_vel_;
	void commandCallback(const bilbot_msgs::Drive::ConstPtr& cmd_vel);
	void stateCallback(const sensor_msgs::JointState::ConstPtr& curr_vel);

public:
	motor_controller(bool side, int gpioA, int gpioB, float kp, float kd, float ki);
	~motor_controller();
	void set_error();
	float control();
};

}

#endif
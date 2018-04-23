#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "math.h"
#include <pigpiod_if2.h>
#include <stdint.h>
#include "sensor_msgs/JointState.h"
#include "bilbot_msgs/Drive.h"

namespace bilbot_hardware {

class motor_controller
{
private:
	float kp_ = 32.0;
	float kd_ = 12.0;
	float ki_ = 30.0; //controller gains

	//Error
	float error_, error_old_;

	//Derivative Variables
	float err_d_, err_d_old1_, err_d_old2_;

	//Integral Variables
	float err_i_, err_i_old_;
	float integral_threshold_ = 3.0;

	//Gpio pins
	int gpioA, gpioB;

	float wheel_cmd_, wheel_vel_;

	int side_;

	void set_wheel_cmd(float wheel_cmd);
	void set_wheel_vel(float wheel_vel);



public:
	motor_controller(int pi, int side, int gpioA, int gpioB);
	~motor_controller();
	
	void filter_velocity();
	void estimate_integral();

	void commandCallback(const bilbot_msgs::Drive::ConstPtr& cmd_vel);
	void stateCallback(const sensor_msgs::JointState::ConstPtr& curr_vel);

	void set_error();
	float control();

};

}

#endif
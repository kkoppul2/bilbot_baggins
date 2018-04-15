#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "math.h"

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


	void filter_velocity();
	void estimate_integral();

	void commandCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
	void stateCallback(const sensor_msgs::JointState::ConstPtr& curr_vel);

public:
	motor_controller(int gpioA, int gpioB, float kp, float kd, float ki);
	~motor_controller();
	void set_error(float err);
	float control();
};

}

#endif
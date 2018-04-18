#include "ros/ros.h"
#include <pigpiod_if2.h>
#include "bilbot_hardware/motor_controller.hpp"
#include "sensor_msgs/JointState.h"

using namespace bilbot_hardware;

int main(int argc, char **argv) {
	ros::init(argc, argv, "motor_output");

	ros::NodeHandle n;

	std::string cmd_topic, curr_topic;
	n.getParam("command_topic", cmd_topic);
	n.getParam("state_topic", curr_topic);

	int pinA, pinB, side;
	n.getParam("pinA", pinA);
	n.getParam("pinB", pinB);
	n.getParam("side", side);

	motor_controller mc(side, pinA, pinB, 1.0, 0.0, 0.0);

	ros::Subscriber cmd = n.subscribe(cmd_topic, 10, &motor_controller::commandCallback, &mc);

	ros::Subscriber curr = n.subscribe(curr_topic, 10, &motor_controller::stateCallback, &mc);

	//Initialize pigpio library
	int pi;
	if (gpioInitialise() < 0) return 1;
	//Create motor controller class;

	float motor_u;

	ros::Rate loop(100);

	while(ros::ok()) {
		//Calculate current error
		mc.set_error();

		//Calculate Integral and Derivative values of error signal 
		mc.filter_velocity();
		mc.estimate_integral();
		
		//Calculate Motor control signal
		motor_u = mc.control();

		//Gpio output 
		if (motor_u >= 0)
		{
			gpioPWM(pinA, motor_u);
			gpioPWM(pinB, 0);
		} else {
			gpioPWM(pinA, 0);
			gpioPWM(pinB, motor_u);
		}

		//Ros Looping
		ros::spinOnce();
		loop.sleep();
	}

	gpioTerminate();

	return 0;
}
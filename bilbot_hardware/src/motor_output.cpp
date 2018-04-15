#include "ros/ros.h"
#include <pigpio.h>
#include "bilbot_hardware/motor_controller.hpp"
#include "bilbot_msgs/Drive"
#include "sensor_msgs/JointState.h"

using namespace bilbot_hardware;

int main(int argc, char **argv) {
	ros::init(argc, argv, "motor_output");

	ros::NodeHandle n;

	std::string cmd_topic, curr_topic;
	n.param("command_topic", cmd_topic, "wheel_vel/right");
	n.param("state_topic", curr_topic, "wheel_state/right");

	int pinA, pinB, side;
	n.param("pinA", pinA, 23);
	n.param("pinB", pinB, 24);
	n.param("side", side, 0);

	motor_controller mc(side, pinA, pinB, 1.0, 0.0, 0.0);

	ros::Subscriber cmd = n.subscribe(cmd_topic, 10, &motor_controller::commandCallback, &mc);

	ros::Subscriber curr = n.subscribe(curr_topic, 10, &motor_controller::stateCallback, &mc);

	//Initialize pigpio library
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
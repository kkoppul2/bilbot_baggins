#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "motor_controller.hpp"
#include <pigpio.h>
#include "motor_controller.hpp"

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

	ros::Subscriber cmd = n.subscribe(cmd_topic, 10, motor_controller::commandCallback);

	ros::Subscriber curr = n.subscribe(curr_topic, 10, motor_controller::stateCallback);

	//Initialize pigpio library
	if (gpioInitialise() < 0) return 1;
	//Create motor controller class;
	motor_controller m(side, pinA, pinB, 1.0, 0.0, 0.0);

	float motor_u;
	float  wheel_cmd, wheel_vel;

	ros::Rate loop(100);

	while(ros::ok()) {
		//Calculate current error
		m.set_error(wheel_cmd - wheel_vel);

		//Calculate Integral and Derivative values of error signal 
		m.filter_velocity();
		m.estimate_integral();
		
		//Calculate Motor control signal
		motor_u = m.control();

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
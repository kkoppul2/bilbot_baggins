#include "ros/ros.h"
#include "bilbot_hardware/motor_controller.hpp"
#include "sensor_msgs/JointState.h"
#include "bilbot_msgs/Drive.h"

#include <pigpiod_if2.h>
#include <unistd.h>
#include <sstream>

using namespace bilbot_hardware;

int main(int argc, char **argv) {
	ros::init(argc, argv, "motor_output");

	ros::NodeHandle n;

	int pinA, pinB, side;
	n.param("pinA", pinA, 12);
	n.param("pinB", pinB, 13);
	n.param("side", side, 0);

	//Initialize pigpio library
	int pi;
	if ((pi = pigpio_start(0,0)) < 0) {
		fprintf(stderr, "pigpio initialisation failed (%d).\n", pi);
		return 1;
	}
	//Create motor controller class;
	motor_controller mc(pi, side, pinA, pinB, 1.0, 0.0, 0.0);

	ros::Subscriber cmd = n.subscribe("cmd_drive", 10, &motor_controller::commandCallback, &mc);

	ros::Subscriber curr = n.subscribe("wheel_state/combined", 10, &motor_controller::stateCallback, &mc);

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
			set_PWM_dutycycle(pi, pinA, motor_u);
			set_PWM_dutycycle(pi, pinB, 0);
		} else {
			set_PWM_dutycycle(pi, pinA, 0);
			set_PWM_dutycycle(pi, pinB, motor_u);
		}

		//Ros Looping
		ros::spinOnce();
		loop.sleep();
	}

	pigpio_stop(pi);

	return 0;
}
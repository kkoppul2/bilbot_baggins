#include "ros/ros.h"
#include "bilbot_hardware/motor_controller.hpp"
#include "sensor_msgs/JointState.h"
#include "bilbot_msgs/Drive.h"
#include "std_msgs/Float64.h"

#include <pigpiod_if2.h>
#include <unistd.h>
#include <sstream>
#include <math.h>

using namespace bilbot_hardware;

int main(int argc, char **argv) {
	ros::init(argc, argv, "motor_output");

	ros::NodeHandle n;

	ros::NodeHandle nh("~");

	int pinA, pinB, side;
	nh.getParam("pinA", pinA);
	nh.getParam("pinB", pinB);
	nh.getParam("side", side);

	//Initialize pigpio library
	int pi;
	if ((pi = pigpio_start(0,0)) < 0) {
		fprintf(stderr, "pigpio initialisation failed (%d).\n", pi);
		return 1;
	}
	//Create motor controller class;
	motor_controller mc(1.0, 0.0, 0.0, pi, side, pinA, pinB);

	ros::Publisher init_cmd_pub = n.advertise<bilbot_msgs::Drive>("cmd_drive", 1, true);

	ros::Publisher motor_out_pub = n.advertise<std_msgs::Float64>("motor_commands", 1);

	bilbot_msgs::Drive init_cmd;
 	init_cmd.drivers[0] = 0.0;
 	init_cmd.drivers[1] = 0.0;
 	init_cmd_pub.publish(init_cmd);

	ros::Subscriber cmd = n.subscribe("cmd_drive", 10, &motor_controller::commandCallback, &mc);

	ros::Subscriber curr = n.subscribe("wheel_state/combined", 10, &motor_controller::stateCallback, &mc);

	float motor_u;

	std_msgs::Float64 cmd_out;

	ros::Rate loop(100);

	while(ros::ok()) {
		std_msgs::Float64 motor_out;
		//Calculate current error
		mc.set_error();

		//Calculate Integral and Derivative values of error signal 
		mc.filter_velocity();
		mc.estimate_integral();
		
		//Calculate Motor control signal
		motor_u = mc.control();

		motor_out.data = motor_u;

		motor_out_pub.publish(motor_out);

		set_PWM_dutycycle(pi, pinA, 0);
		set_PWM_dutycycle(pi, pinB, 0);

		// //Gpio output 
		// if (motor_u >= 0)
		// {
		// 	set_PWM_dutycycle(pi, pinA, motor_u);
		// 	set_PWM_dutycycle(pi, pinB, 0);
		// } else {
		// 	set_PWM_dutycycle(pi, pinA, 0);
		// 	set_PWM_dutycycle(pi, pinB, fabs(motor_u));
		// }

		//Ros Looping
		ros::spinOnce();
		loop.sleep();
	}

	pigpio_stop(pi);

	return 0;
}
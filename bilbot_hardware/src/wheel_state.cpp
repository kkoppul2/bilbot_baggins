#include <pigpiod_if2.h>
#include <unistd.h>

#include "bilbot_hardware/rotary_encoder.hpp"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

using namespace bilbot_hardware;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "wheel_state");

	ros::NodeHandle n;

	int pinA, pinB;
	n.param("pinA", pinA, 7);
	n.param("pinB", pinB, 8);

	ros::Publisher wheel_state = n.advertise<sensor_msgs::JointState>("wheel_state", 100);

	ros::Rate loop(100);
	// int pi;
	// ROS_INFO("Before Pigpio init");
	// if ((pi = pigpio_start(0,0)) < 0) {
	// 	fprintf(stderr, "pigpio initialisation failed (%d).\n", pi);
	// 	return 1;
	// }

	// re_decoder dec(pi, pinA, pinB);

	while (ros::ok()){
		sensor_msgs::JointState wheel;
		wheel.header.stamp = ros::Time::now();
		wheel.header.frame_id = "/world";

		//Publish position and velocity to JointState message
		wheel.position[0] = 0.0; //dec.getPosition();
		wheel.velocity[0] = 0.0; //dec.getVelocity();
		wheel.effort[0] = 0.0;

		wheel_state.publish(wheel);

		ros::spinOnce();

		loop.sleep();
	}
	//release GPIO resources
	// dec.re_cancel();
	//end gpio use functionality
	// pigpio_stop(pi);

	return 0;
}


#include "ros/ros.h"
#include "bilbot_hardware/rotary_encoder.hpp"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include <pigpiod_if2.h>
#include <sstream>
#include <unistd.h>

using namespace bilbot_hardware;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheel_state_node");

	ros::NodeHandle n;

	ros::NodeHandle nh("~");


	int pinA, pinB;
	std::string wheel_name;
	nh.getParam("wheel_name", wheel_name);
	nh.getParam("pinA", pinA);
	nh.getParam("pinB", pinB);

	ros::Publisher wheel_state = n.advertise<sensor_msgs::JointState>("wheel_state", 1);

	ros::Rate loop(100);
	int pi;
	if ((pi = pigpio_start(0,0)) < 0) {
		fprintf(stderr, "pigpio initialisation failed (%d).\n", pi);
		return 1;
	}

	re_decoder dec(pi, pinA, pinB);

	while (ros::ok()){
		sensor_msgs::JointState wheel;

		wheel.name.resize(1);
		wheel.position.resize(1);
		wheel.velocity.resize(1);

		wheel.header.stamp = ros::Time::now();
		wheel.header.frame_id = "/world";
		// // //Publish position and velocity to JointState message
		wheel.name[0] = wheel_name;
		wheel.position[0] = dec.getPosition();
		wheel.velocity[0] = dec.getVelocity();

		// ROS_INFO("Reached the loop");
		wheel_state.publish(wheel);

		ros::spinOnce();

		loop.sleep();
	}
	//release GPIO resources
	dec.re_cancel();
	//end gpio use functionality
	pigpio_stop(pi);

	return 0;
}


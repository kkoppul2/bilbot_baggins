#include <pigpio.h>
#include <unistd.h>

#include "../include/rotary_encoder.hpp"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

using namespace bilbot_drive;

void callback(int way)
{
   static float pos = 0;

   pos += way;

}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "wheel_state")

	ros::NodeHandle n;

	int pinA, pinB;
	n.param("pinA", pinA, 7);
	n.param("pinB", pinB, 8);

	ros::Publisher wheel_state = n.advertise<sensor_msgs::JointState>("wheel_state", 100);

	ros::Rate loop(100);

	float position, position_old, velocity, velocity_old1, velocity_old2; //Variables used to implement IIR filter

	float resolution = 0.1308997; //Angular resolution per encoder tick

	if (gpioInitialise() < 0) return 1;

	re_decoder dec(pinA, pinB, callback);

	while (ros::ok()){
		sensor_msgs::JointState wheel;
		wheel.header.stamp = ros::Time::now();
		wheel.header.frame_id = "/world";

		//Take position reading from encoder and find angular position
		position = pos*resolution;

		//IIR filter on velocity
		velocity = (position - position_old)/0.01;
		velocity = (velocity + velocity_old1 + velocity_old2)/3.0;

		position_old = position;
		velocity_old2 = velocity_old1;
		velocity_old1 = velocity;

		//Publish position and velocity to JointState message
		wheel.position = position;
		wheel.velocity = velocity;
		wheel.effort = 0.0;

		wheel_state.publish(wheel);

		ros::spinOnce();

		loop.sleep();
	}
	//release GPIO resources
	dec.re_cancel();
	//end gpio use functionality
	gpioTerminate();

	return 0;
}


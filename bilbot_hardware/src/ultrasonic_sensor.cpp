#include "ros/ros.h"
#include "bilbot_hardware/ultrasonic.hpp"
#include "sensor_msgs/Range.h"

#include <pigpiod_if2.h>
#include <unistd.h>
#include <math.h>

using namespace bilbot_hardware;

int main(int argc, char** argv) {
	ros::init(argc, argv, "ultrasonic_output");

	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	int pinA, pinB;
	nh.getParam("Trig", pinA);
	nh.getParam("Echo", pinB);

	std::string pub_topic;
	nh.getParam("topic_name", pub_topic);

	int pi;
	if ((pi = pigpio_start(0,0)) < 0) {
		fprintf(stderr, "pigpio initialisation failed (%d).\n", pi);
		return 1;
	}

	ultrasonic u(pi, pinA, pinB);

	ros::Publisher sensor_output = n.advertise<sensor_msgs::Range>("ultrasonic", 1);

	ros::Rate loop(10);

	while(ros::ok()) {

		sensor_msgs::Range out;
		out.header.stamp = ros::Time::now();
		out.header.frame_id = "/ultrasonic";

		out.radiation_type = 0;
		out.field_of_view = 0.261799;
		out.min_range = 0.02;
		out.max_range = 4.5;

		out.range = u.getRange();;

		sensor_output.publish(out);

		ros::spinOnce();
		loop.sleep();
	}

	pigpio_stop(pi);
}
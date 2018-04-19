#include "ros/ros.h"
#include <boost/assign.hpp>
#include "bilbot_hardware/merge.hpp"
#include "sensor_msgs/JointState.h"



int main(int argc, char **argv) {
	ros::init(argc, argv, "merger");

	ros::NodeHandle n;

	merger m;

	ros::Subscriber right = n.subscribe("wheel_state/right", 1, merger::rightCallback, &m);
	ros::Subscriber left = n.subscribe("wheel_state/left", 1, merger::leftCallback, &m);

	ros::Publisher merged = n.advertise<sensor_msgs::JointState>("wheel_state/combined", 1);

	ros::Rate loop(100);

	while (ros::ok()) {
		sensor_msgs::JointState merged_message;
		merged_message.header.stamp = ros::Time::now();
		merged_message.header.frame_id = "/world";

		merged_message.name.resize(2);
		merged_message.position.resize(2);
		merged_message.velocity.resize(2);

		merged_message.name[0] = ""; //right_wheel.name[0];
		merged_message.name[1] = ""; //left_wheel.name[0];
		merged_message.position[0] = 0.0; //right_wheel.position[0];
		merged_message.position[1] = 0.0; //left_wheel.position[0];
		merged_message.velocity[0] = 0.0; //right_wheel.velocity[0];
		merged_message.velocity[1] = 0.0; //left_wheel.velocity[0];

		merged.publish(merged_message);

		ros::spinOnce();
		loop.sleep();
	}


	return 0;
}
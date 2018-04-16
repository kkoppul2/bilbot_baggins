#include "ros/ros.h"
#include <boost/assign.hpp>
// #include "bilbot_hardware/merge.hpp"
#include "sensor_msgs/JointState.h"

sensor_msgs::JointState right_wheel, left_wheel;

void rightCallback(const sensor_msgs::JointState::ConstPtr& rmsg) {
	right_wheel.position = rmsg->position;
	right_wheel.velocity = rmsg->velocity;
}

void leftCallback(const sensor_msgs::JointState::ConstPtr& lmsg) {
	left_wheel.position = lmsg->position;
	left_wheel.velocity = lmsg->velocity;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "merger");

	ros::NodeHandle n;

	ros::Subscriber right = n.subscribe("wheel_state/right", 1, rightCallback);
	ros::Subscriber left = n.subscribe("wheel_state/left", 1, leftCallback);

	ros::Publisher merged = n.advertise<sensor_msgs::JointState>("wheel_state/combined", 1);

	ros::Rate loop(100);
	ROS_INFO("Outside of loop");
	while (ros::ok()) {
		sensor_msgs::JointState merged_message;
		merged_message.header.stamp = ros::Time::now();
		merged_message.header.frame_id = "/world";

		ROS_INFO("Inside of loop-1");
		merged_message.name = {"right_wheel_joint", "left_wheel_joint"};
		// merged_message.position = {right_wheel.position[0], left_wheel.position[0]};
		// merged_message.velocity = {right_wheel.velocity[0], left_wheel.velocity[0]};
		ROS_INFO("Inside of loop-2");
		merged.publish(merged_message);

		ros::spinOnce();
		loop.sleep();
	}


	return 0;
}
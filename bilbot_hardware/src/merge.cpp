#include "ros/ros.h"
#include "bilbot_hardware/merge.hpp"
#include "sensor_msgs/JointState.h"

using namespace bilbot_hardware;

void merger::rightCallback(const sensor_msgs::JointState::ConstPtr& rmsg) {
	right_wheel.position[0] = rmsg->position[0];
	right_wheel.velocity[0] = rmsg->velocity[0];
	right_wheel.effort[0] = 0.0;
}

void merger::leftCallback(const sensor_msgs::JointState::ConstPtr& lmsg) {
	left_wheel.position[0] = lmsg->position[0];
	left_wheel.velocity[0] = lmsg->velocity[0];
	left_wheel.effort[0] = 0.0;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "merger");

	ros::NodeHandle n;

	merger m;

	ros::Subscriber right = n.subscribe("wheel_state/right", 1, &merger::rightCallback, &m);
	ros::Subscriber left = n.subscribe("wheel_state/left", 1, &merger::leftCallback, &m);

	ros::Publisher merged = n.advertise<sensor_msgs::JointState>("wheel_state/combined", 1);

	ros::Rate loop(100);

	while (ros::ok()) {
		sensor_msgs::JointState merged_message;

		merged_message.position[0] = m.right_wheel.position[0];
		merged_message.velocity[0] = m.right_wheel.velocity[0];
		merged_message.effort[0] = m.right_wheel.effort[0];

		merged_message.position[1] = m.left_wheel.position[0];
		merged_message.velocity[1] = m.left_wheel.velocity[0];
		merged_message.effort[1] = m.left_wheel.effort[0];

		merged.publish(merged_message);

		ros::spinOnce();
		loop.sleep();
	}


	return 0;
}
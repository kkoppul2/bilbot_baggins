#include "ros/ros.h"
#include "bilbot_hardware/merge.hpp"
#include "sensor_msgs/JointState.h"

using namespace bilbot_hardware;

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

		merged_message.position[0] = m.right.position[0];
		merged_message.velocity[0] = m.right.velocity[0];
		merged_message.effort[0] = m.right.effort[0];

		merged_message.position[1] = m.left.position[0];
		merged_message.velocity[1] = m.left.velocity[0];
		merged_message.effort[1] = m.left.effort[0];

		merged.publish(merged_message);

		ros::spinOnce();
		loop.sleep();
	}


	return 0;
}
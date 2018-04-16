#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

using namespace bilbot_hardware;

void rightCallback(const sensor_msgs::JointState::ConstPtr& msg) {
	right_wheel = *msg;
}

void leftCallback(const sensor_msgs::JointState::ConstPtr& msg) {
	left_wheel = *msg;	
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "merger");

	ros::NodeHandle n;

	ros::Subscriber right = n.subscribe("wheel_state/right", 10, rightCallback);
	ros::Subscriber left = n.subscribe("wheel_state/left", 10, leftCallback);

	ros::Publisher merged = n.advertise<sensor_msgs::JointState>("wheel_state/combined", 10);

	ros::Rate loop(100);

	sensor_msgs::JointState right_wheel, left_wheel;

	while (ros::ok()) {
		sensor_msgs::JointState merged_message;

		merged_message.position[0] = right_wheel.position[0];
		merged_message.velocity[0] = right_wheel.velocity[0];
		merged_message.effort[0] = right_wheel.effort[0];

		merged_message.position[1] = left_wheel.position[0];
		merged_message.velocity[1] = left_wheel.velocity[0];
		merged_message.effort[1] = left_wheel.effort[0];

		merged.publish(merged_message);

		ros::spinOnce();
		loop.sleep();
	}


	return 0;
}
#ifndef MERGE_H_
#define MERGE_H_ 

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

namespace bilbot_hardware {

class merger
{
public:
	sensor_msgs::JointState right_wheel;
	sensor_msgs::JointState left_wheel;

	void rightCallback(const sensor_msgs::JointState::ConstPtr& rmsg);
	void leftCallback(const sensor_msgs::JointState::ConstPtr& lmsg);
};

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
}

#endif
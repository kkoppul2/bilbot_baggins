#ifndef MERGE_H_
#define MERGE_H_ 

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

namespace bilbot_hardware {

class merger
{
public:
	sensor_msgs::JointState right, left;

	void rightCallback(const sensor_msgs::JointState::ConstPtr& rmsg);
	void leftCallback(const sensor_msgs::JointState::ConstPtr& lmsg);
};

void merger::rightCallback(const sensor_msgs::JointState::ConstPtr& rmsg)
	right.position[0] = rmsg->position[0];
	right.velocity[0] = rmsg->velocity[0];
	right.effort[0] = 0.0;
}

void merger::leftCallback(const sensor_msgs::JointState::ConstPtr& lmsg) {
	left.position[0] = lmsg->position[0];
	left.velocity[0] = lmsg->velocity[0];
	left.effort[0] = 0.0;
}

#endif
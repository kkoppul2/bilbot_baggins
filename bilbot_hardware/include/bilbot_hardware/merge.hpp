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


}

#endif
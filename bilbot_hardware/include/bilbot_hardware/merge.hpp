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

	void left_init();
	void right_init();

};

void merger::right_init() {
	right_wheel.name.resize(1);
	right_wheel.position.resize(1);
	right_wheel.velocity.resize(1);

	right_wheel.name[0] = "right_wheel_joint";
	right_wheel.position[0] = 0.0;
	right_wheel.velocity[0] = 0.0;
}

void merger::left_init() {
	left_wheel.name.resize(1);
	left_wheel.position.resize(1);
	left_wheel.velocity.resize(1);

	left_wheel.name[0] = "left_wheel_joint";
	left_wheel.position[0] = 0.0;
	left_wheel.velocity[0] = 0.0;
}

void merger::rightCallback(const sensor_msgs::JointState::ConstPtr& rmsg) {
	right_wheel = *rmsg;
}

void merger::leftCallback(const sensor_msgs::JointState::ConstPtr& lmsg) {
	left_wheel = *lmsg;
}

}

#endif
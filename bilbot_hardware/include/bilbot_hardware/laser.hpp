#ifndef LASER_H_
#define LASER_H_

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

namespace bilbot_hardware {

class laser
{
public:
	sensor_msgs::Range left_sensor;
	sensor_msgs::Range middle_sensor;
	sensor_msgs::Range right_sensor;

	void lsonarCallback(const sensor_msgs::Range::ConstPtr& msg);
	void msonarCallback(const sensor_msgs::Range::ConstPtr& msg);
	void rsonarCallback(const sensor_msgs::Range::ConstPtr& msg);
	
};

void laser::lsonarCallback(const sensor_msgs::Range::ConstPtr& msg) {
	left_sensor = *msg;
}

void laser::msonarCallback(const sensor_msgs::Range::ConstPtr& msg) {
	middle_sensor = *msg;
}

void laser::rsonarCallback(const sensor_msgs::Range::ConstPtr& msg) {
	right_sensor = *msg;
}

}

#endif
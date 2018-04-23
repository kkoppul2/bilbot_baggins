#ifndef NAV_GOALS_H_
#define NAV_GOALS_H_

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

namespace bilbot_nav {

class nav_goals
{
public:
	geometry_msgs::PoseStamped goal_;

	void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	
};

void nav_goals::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	goal_ = *msg;
}

}

#endif
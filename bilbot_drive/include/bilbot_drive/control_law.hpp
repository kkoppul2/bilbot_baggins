#ifndef CONTROL_H_
#define CONTROL_H_ 

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "math.h"


namespace bilbot_drive {

class controller
{
public:
	geometry_msgs::PoseStamped goal_pose;
	nav_msgs::Odometry current_pose;

	float ka;
	float kp;

	float err_d;

	float err_t;
	
	void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void currCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void configCallback(bilbot_drive::Pos_ControllerConfig &config, uint32_t level);

	float angular();

	float linear();
	
};

void controller::angular() {
	return kp*sin(err_t)*cos(err_t)+ ka*err_t;
}

float controller::linear() {
	return kp*err_d*cos(err_t);
}

void controller::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	goal_pose = *msg;
}

void controller::currCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	current_pose = *msg;
}

void controller::configCallback(bilbot_drive::Pos_ControllerConfig &config, uint32_t level) {
	ka = config.ka;
	kp = config.kp;
}

}



#endif
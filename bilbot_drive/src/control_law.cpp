#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <bilbot_drive/Pos_ControllerConfig.h>
#include "bilbot_drive/control_law.hpp"
#include <dynamic_reconfigure/server.h>
#include "math.h"
#include <tf/transform_datatypes.h>


using namespace bilbot_drive;

int main(int argc, char** argv) {
	ros::init(argc, argv, "simple_control");

	ros::NodeHandle n;

	controller c;

	ros::Subscriber goal_pose = n.subscribe("goal_position", 1, &controller::goalCallback, &c);
	ros::Subscriber curr_pose = n.subscribe("bilbot_diff_controller/odom", 1, &controller::currCallback, &c);

	ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("bilbot_diff_controller/cmd_vel", 1);

	dynamic_reconfigure::Server<bilbot_drive::Pos_ControllerConfig> server;
	dynamic_reconfigure::Server<bilbot_drive::Pos_ControllerConfig>::CallbackType f;

	f = boost::bind(&controller::configCallback, &c, _1, _2);
	server.setCallback(f);

	ros::Rate loop(50);

	while (ros::ok()) {
		geometry_msgs::Twist velocity_commands;

		c.err_d = sqrt(pow(c.goal_pose.pose.position.x - c.current_pose.pose.pose.position.x, 2) + 
			pow(c.goal_pose.pose.position.y - c.current_pose.pose.pose.position.y, 2) + 
			pow(c.goal_pose.pose.position.z - c.current_pose.pose.pose.position.z, 2));

		tf::Quaternion goal(c.goal_pose.pose.orientation.x, c.goal_pose.pose.orientation.y, c.goal_pose.pose.orientation.z, c.goal_pose.pose.orientation.w);
		tf::Matrix3x3 g(goal);
		float gr, gp, gy;
		g.getRPY(gr, gp, gy);

		tf::Quaternion current(c.current_pose.pose.pose.orientation.x, c.current_pose.pose.pose.orientation.y, c.current_pose.pose.pose.orientation.z, c.current_pose.pose.pose.orientation.w);
		tf::Matrix3x3 curr(current);
		float cr, cp, cy;
		curr.getRPY(cr, cp, cy);
		c.err_t = gy - cy;


		velocity_commands.linear.x = c.linear();
		velocity_commands.linear.y = 0;
		velocity_commands.linear.z = 0;

		velocity_commands.angular.x = 0;
		velocity_commands.angular.y = 0;
		velocity_commands.angular.z = c.angular();

		cmd_vel.publish(velocity_commands);

		ros::spinOnce();
		loop.sleep();
	}
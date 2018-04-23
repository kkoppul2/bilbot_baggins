#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "bilbot_nav/nav_goals.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"

// #include <tf/transform_broadcaster.h>

using namespace bilbot_nav;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle n;

  // tf::TransformBroadcaster br;
  // tf::Transform transform;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  nav_goals nav;

  ros::Subscriber goal_sub = n.subscribe("goal_position", 1, &nav_goals::goalCallback, &nav);

  ros::Publisher action_status_pub = n.advertise<std_msgs::Bool>("action_status", 1);

  ros::Rate loop(2);

  while (ros::ok()) {
    // transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    // transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = nav.goal_;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    std_msgs::Bool status;
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      status.data = true;
    } else {
      status.data = false;
    }
    action_status_pub.publish(status);

    ros::spinOnce();
    loop.sleep();

  }
 
  return 0;
}
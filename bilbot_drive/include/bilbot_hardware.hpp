#ifndef BILBOT_HARDWARE_H
#define BILBOT_HARDWARE_H

#include "boost/thread.hpp"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"

#include "bilbot_msgs/Drive.h"

#include "realtime_tools/realtime_publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


namespace bilbot_hardware
{

class BilbotHardware : public hardware_interface::RobotHW
{
public:
  BilbotHardware();
  void copyJointsFromHardware();
  void publishDriveFromController();

private:
  void rightCallback(const sensor_msgs::JointState::ConstPtr& right);
  void leftCallback(const sensor_msgs::JointState::ConstPtr& left);

  ros::NodeHandle nh_;
  ros::Subscriber feedback_right_;
  ros::Subscriber feedback_left_;
  realtime_tools::RealtimePublisher<bilbot_msgs::Drive> cmd_drive_pub_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  // These are mutated on the controls thread only.
  struct Joint
  {
    double position;
    double velocity;
    double effort;
    double velocity_command;

    Joint() : position(0), velocity(0), effort(0), velocity_command(0)
    {
    }
  }
  joints_[2];

  // This pointer is set from the ROS thread.
  jackal_msgs::Feedback::ConstPtr feedback_msg_;
  boost::mutex feedback_msg_mutex_;
};

}  // namespace bilbot_hardware

#endif // BILBOT_HARDWARE_H
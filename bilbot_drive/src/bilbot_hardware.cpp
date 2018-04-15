#include <boost/assign.hpp>
#include "jackal_base/jackal_hardware.h"

namespace bilbot_hardware
{

BilbotHardware::BilbotHardware()
{
  ros::V_string joint_names = boost::assign::list_of("right_wheel_joint")
      ("left_wheel_joint");

  for (unsigned int i = 0; i < joint_names.size(); i++) {
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
        &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  feedback_right_ = nh_.subscribe("wheel_state/right", 1, &BilbotHardware::rightCallback, this);
  feedback_right_ = nh_.subscribe("wheel_state/right", 1, &BilbotHardware::rightCallback, this);

  // Realtime publisher, initializes differently from regular ros::Publisher
  cmd_drive_pub_.init(nh_, "cmd_drive", 1);
}

/**
 * Populates the internal joint state struct from the most recent Feedback message
 * received from the MCU.
 *
 * Called from the controller thread.
 */
void BilbotHardware::copyJointsFromHardware()
{
  boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock);
  if (feedback_msg_ && feedback_msg_lock)
  {
    for (int i = 0; i <2; i++)
    {
      joints_[i].position = feedback_msg_->drivers[i % 2].measured_travel;
      joints_[i].velocity = feedback_msg_->drivers[i % 2].measured_velocity;
      joints_[i].effort = 0;  // TODO(mikepurvis): determine this from amperage data.
    }
  }
}

/**
 * Populates and publishes Drive message based on the controller outputs.
 *
 * Called from the controller thread.
 */
void BilbotHardware::publishDriveFromController()
{
  if (cmd_drive_pub_.trylock())
  {
    cmd_drive_pub_.msg_.right_cmd = joints_[0].velocity_command;
    cmd_drive_pub_.msg_.left_cmd = joints_[1].velocity_command;
    cmd_drive_pub_.unlockAndPublish();
  }
}

void BilbotHardware::rightCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // Update the feedback message pointer to point to the current message. Block
  // until the control thread is not using the lock.
  boost::mutex::scoped_lock lock(feedback_msg_mutex_);
  feedback_msg_ = msg;
}

void BilbotHardware::rightCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // Update the feedback message pointer to point to the current message. Block
  // until the control thread is not using the lock.
  boost::mutex::scoped_lock lock(feedback_msg_mutex_);
  feedback_msg_ = msg;
}

} // namespace bilbot_hardware
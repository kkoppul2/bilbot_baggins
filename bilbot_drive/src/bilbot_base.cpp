#include <string>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include "controller_manager/controller_manager.h"
#include "bilbot_drive/bilbot_hardware.hpp"
#include "ros/ros.h"

typedef boost::chrono::steady_clock time_source;

void controlThread(ros::Rate rate, bilbot_drive::BilbotHardware* robot, controller_manager::ControllerManager* cm)
{
  time_source::time_point last_time = time_source::now();

  while (1)
  {
    // Calculate monotonic time elapsed
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    robot->copyJointsFromHardware();
    cm->update(ros::Time::now(), elapsed);
    robot->publishDriveFromController();
    rate.sleep();
  }
}

int main(int argc, char* argv[])
{
  // Initialize ROS node.
  ros::init(argc, argv, "bilbot_node");
  bilbot_drive::BilbotHardware bilbot;

  // Background thread for the controls callback.
  ros::NodeHandle controller_nh("");
  controller_manager::ControllerManager cm(&bilbot, controller_nh);
  boost::thread(boost::bind(controlThread, ros::Rate(50), &bilbot, &cm));

  // Foreground ROS spinner for ROS callbacks, including rosserial, diagnostics
  ros::spin();

  return 0;
}
#include "ros/ros.h"
#include <boost/assign.hpp>
#include "bilbot_hardware/laser.hpp"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_broadcaster.h>

using namespace bilbot_hardware;

int main(int argc, char** argv) {
	ros::init(argc, argv, "laser");

	ros::NodeHandle n;

	laser l;

	ros::Subscriber left = n.subscribe("ultrasonic/left", 1, &laser::lsonarCallback, &l);
	ros::Subscriber middle = n.subscribe("ultrasonic/middle", 1, &laser::msonarCallback, &l);
	ros::Subscriber right = n.subscribe("ultrasonic/right", 1, &laser::rsonarCallback, &l);

	ros::Publisher laser = n.advertise<sensor_msgs::LaserScan>("fakelaser", 1);

	tf::TransformBroadcaster broadcaster;

	ros::Rate loop(100);

	float offset = 1.0;

	while (ros::ok()) {
		//Publish tf transform between "laser" and base
		broadcaster.sendTransform(tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.01, 0.0, 0.088)), 
			ros::Time::now(), "base_link", "laser"));

		//Publish laser message as aggregate of ultrasonic sensors
		sensor_msgs::LaserScan scan;
		scan.header.stamp = ros::Time::now();
		scan.header.frame_id = "laser";
		scan.angle_min = -0.523599;
		scan.angle_max = 0.523599;
		scan.angle_increment = 0.523599;
		scan.time_increment = 0;
		scan.scan_time = 0.01;
		scan.range_min = 0.02;
		scan.range_max = 4.5;

		scan.ranges.resize(3);

		scan.ranges[0] = l.right_sensor.range + 0.13;
		scan.ranges[1] = l.middle_sensor.range + 0.13;
		scan.ranges[2] = l.left_sensor.range + 0.13;

		laser.publish(scan);

		ros::spinOnce();
		loop.sleep();
	}
}
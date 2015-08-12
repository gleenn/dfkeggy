#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void broadcast_geometry() {
	static ros::NodeHandle n;
	static tf::TransformBroadcaster broadcaster;

	    broadcaster.sendTransform(
	      tf::StampedTransform(
	        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
	        ros::Time::now(),"keggy_base", "keggy_kinect"));
}
#include <cmath>
#include <ros/ros.h>

#include "dfcompass_msgs/status.h"
#include "roboteq_msgs/Command.h"
#include "sensor_msgs/NavSatFix.h"
#include "roboteq_msgs/Command.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "simulator");
    ros::NodeHandle ros_node;
	ros::Rate loop_rate(1);

    ros::Publisher  gps_pub = ros_node.advertise<sensor_msgs::NavSatFix>("/gps_driver/fix", 1000);
    
    int i = 0;
    //double x = y = t = 0;

 	while(ros::ok()) {

 		sensor_msgs::NavSatFix msg_fix;
 		msg_fix.longitude = -122.01916666666666;
 		msg_fix.latitude = 37.4105 + 0.000001*(i%10);
 		//msg_fix.covariance[0] = msg_fix.covariance[1] = msg_fix.covariance[2] = 0;
 		gps_pub.publish(msg_fix);
 		i++;

		ros::spinOnce();
		loop_rate.sleep();
	}
}
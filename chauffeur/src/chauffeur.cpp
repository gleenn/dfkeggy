#include <cmath>
#include <ros/ros.h>

#include "dfcompass_msgs/status.h"
#include "dfkeggy_webui/UIStatus.h"
#include "roboteq_msgs/Command.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"

inline double normalize_angle(double angle, double center=0) {
    return angle - 2*M_PI*std::floor((angle + M_PI - center)/(2*M_PI));
}

inline double clamp(double val, double amp) {
	return val == 0 || std::fabs(val) <= amp ? val : val/std::fabs(val); 
}

inline double from_degrees(double deg) {
	return deg*M_PI*2/360;
}

const double ROBOTEQ_CMD_MAX = 100;
const double ANGULAR_THRESHOLD = from_degrees(3);
const double LINEAR_THRESHOLD = 5;

ros::Publisher *left_wheel_pub = 0, *right_wheel_pub = 0;

// static void on_compass(const dfcompass_msgs::status::ConstPtr& msg) {
// }

// static void on_gps(const sensor_msgs::NavSatFix::ConstPtr& msg) {
// }

static void drive(double lv, double rv) {
	double lm = clamp(lv, 1) * ROBOTEQ_CMD_MAX;
	double rm = clamp(rv, 1) * ROBOTEQ_CMD_MAX;

	roboteq_msgs::Command cmd;
	cmd.commanded_velocity = lm;
	left_wheel_pub->publish(cmd);

	cmd.commanded_velocity = rm;
	right_wheel_pub->publish(cmd);

	printf("chaffeur: drive L:%+1.2f/%+05.0f R:%+1.2f/%+05.0f\n", lv, lm, rv, rm);
}

static void on_webui(const dfkeggy_webui::UIStatus::ConstPtr& msg) {
	double theta;

	switch(msg->mode) {
		case 'C':
			drive(msg->v_left, msg->v_right);
			break;

		case 'F':
			theta = normalize_angle(msg->goal_theta);
			if (theta > ANGULAR_THRESHOLD) {
				drive(1,-1);
			} else if (theta < -ANGULAR_THRESHOLD) {
				drive(-1, 1);
			} else if (msg->goal_y < -msg->goal_accuracy) {
				drive(1,1);
			} else {
				drive(0,0); 
				printf("chaffeur: arrived\n");
			}
			break;

		default: // 'S'
			drive(0,0);
			break;
	}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "chaffeur");
    ros::NodeHandle ros_node;
	ros::Rate loop_rate(100);

    // ros::Subscriber compass_sub = ros_node.subscribe("/dfcompass_driver/status", 1000, callback_compass);
    // ros::Subscriber gps_sub = ros_node.subscribe("/gps_driver/fix", 1000, callback_gps);
    ros::Subscriber webui_sub = ros_node.subscribe("/webui/status", 1000, on_webui);

    ros::Publisher  lw_pub = ros_node.advertise<roboteq_msgs::Command>("/roboteq_left/cmd", 1000);
    ros::Publisher  rw_pub = ros_node.advertise<roboteq_msgs::Command>("/roboteq_right/cmd", 1000);
 	left_wheel_pub = &lw_pub;
 	right_wheel_pub = &rw_pub;

 	while(ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();
	}
}
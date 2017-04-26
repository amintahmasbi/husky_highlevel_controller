#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

#include <math.h>

//ROS
#include <visualization_msgs/Marker.h>
namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
				nodeHandle_(nodeHandle), twistMsg_() {

	if (!nodeHandle_.getParam("scan_topic", scanTopic_)) {
		ROS_ERROR("Could not read scan topic parameters.");
		ros::requestShutdown();
	}

	if (!nodeHandle_.getParam("control_topic", controlTopic_)) {
		ROS_ERROR("Could not read control topic parameters.");
		ros::requestShutdown();
	}

	if (!nodeHandle_.getParam("controller_p_gain", controller_gain)) {
		ROS_ERROR("Could not read controller gain parameters.");
		ros::requestShutdown();
	}
	publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>(controlTopic_, 1);

	visPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>(
			"visualization_marker", 0);

	subscriber_ = nodeHandle_.subscribe(scanTopic_, 1,
			&HuskyHighlevelController::topicCallback, this);

	ROS_INFO("Successfully launched node.");
}

HuskyHighlevelController::~HuskyHighlevelController() {
}

void HuskyHighlevelController::topicCallback(
		const sensor_msgs::LaserScan::ConstPtr& message) {
	unsigned int num_points = (message->angle_max - message->angle_min)
					/ message->angle_increment;

	double minRange = message->range_max;
	unsigned int minRnage_angleIndex = 0;

	for (unsigned int var = 0; var < num_points; ++var) {

		if (message->ranges[var] < minRange) {
			minRange = message->ranges[var];
			minRnage_angleIndex = var;

		}
	}

	double minRange_angle = message->angle_min
			+ minRnage_angleIndex * message->angle_increment;

	ROS_INFO_STREAM("Range:" << minRange << " Angle: " << minRange_angle);

	visualization_msgs::Marker marker = visualization_msgs::Marker();

	marker.header.frame_id = "base_laser";
	marker.header.stamp = ros::Time();
	marker.ns = "husky_highlevel_controller";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;

	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 1;
	marker.pose.position.x = minRange * cos(minRange_angle);
	marker.pose.position.y = minRange * sin(minRange_angle);

	visPublisher_.publish(marker);

	algorithm_.updateData(minRange, minRange_angle);

	if (minRange < 0.2) {
		twistMsg_.linear.x = 0;
		twistMsg_.angular.z = 0;
	} else {

		twistMsg_.linear.x = 0.5;
		twistMsg_.angular.z = -1 * controller_gain * minRange_angle;
	}
	publisher_.publish(twistMsg_);

}
} /* namespace */

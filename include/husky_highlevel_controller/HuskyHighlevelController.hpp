#pragma once

#include "husky_highlevel_controller/Algorithm.hpp"

//ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

private:
	/*!
	 * Reads and verifies the ROS parameters.
	 * @return true if successful.
	 */
	bool readParameters();

	/*!
	 * ROS topic callback method.
	 * @param message the received message.
	 */
	void topicCallback(const sensor_msgs::LaserScan::ConstPtr& message);


	//! ROS node handle.
	ros::NodeHandle nodeHandle_;

	//! ROS topic subscriber.
	ros::Subscriber subscriber_;

	//! ROS topic Publisher /cmd_vel of robot
	ros::Publisher publisher_;

	//! ROS Visualize Marker  Publisher
	ros::Publisher visPublisher_;

	//! ROS topic name to subscribe to.
	std::string scanTopic_;

	//! ROS control topic
	std::string controlTopic_;

	//! Published Twist message
	geometry_msgs::Twist twistMsg_;

	//! Controller gain
	double controller_gain;

	//! Algorithm computation object.
	Algorithm algorithm_;
};

} /* namespace */

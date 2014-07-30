/*
 * listener_node.cpp
 *
 *  Created on: Jul 30, 2014
 *      Author: dario
 */
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

void odom_no_err_Callback(const nav_msgs::Odometry& msg)
{
	ROS_INFO_STREAM("********************** Heard an ODOM NO ERROR msg **********************");
	ROS_INFO_STREAM("State: ");
	ROS_INFO_STREAM(msg.pose.pose.position);
	ROS_INFO_STREAM(msg.pose.pose.orientation);
	ROS_INFO_STREAM("Speed: ");
	ROS_INFO_STREAM(msg.twist.twist.linear);
	ROS_INFO_STREAM(msg.twist.twist.angular);
}

void odom_Callback(const nav_msgs::Odometry& msg)
{
	ROS_INFO_STREAM("********************** Heard an ODOM msg **********************");
	ROS_INFO_STREAM("State: ");
	ROS_INFO_STREAM(msg.pose.pose.position);
	ROS_INFO_STREAM(msg.pose.pose.orientation);
	ROS_INFO_STREAM("Speed: ");
	ROS_INFO_STREAM(msg.twist.twist.linear);
	ROS_INFO_STREAM(msg.twist.twist.angular);
}

void particle_pose_Callback(const nav_msgs::Odometry& msg)
{
	ROS_INFO_STREAM("********************** Heard a PARTICLE_POSE msg **********************");
	ROS_INFO_STREAM("State: ");
	ROS_INFO_STREAM(msg.pose.pose.position);
	ROS_INFO_STREAM(msg.pose.pose.orientation);
	ROS_INFO_STREAM("Speed: ");
	ROS_INFO_STREAM(msg.twist.twist.linear);
	ROS_INFO_STREAM(msg.twist.twist.angular);
}

int main(int argc, char **argv)
{
	// init ROS and NodeHandle
	ros::init(argc, argv, "listener_node");
	ros::NodeHandle n;

	// init subscriber
	ros::Subscriber sub1 = n.subscribe("visual_odometry/odom_no_error", 1000, odom_no_err_Callback);
	ros::Subscriber sub2 = n.subscribe("visual_odometry/odom", 1000, odom_Callback);
	ros::Subscriber sub3 = n.subscribe("layout_manager/particle_pose",1000,particle_pose_Callback);
	ros::spin();

	return 0;
}

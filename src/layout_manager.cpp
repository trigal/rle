/*
 * layout_manager.cpp
 *
 *  Created on: Jul 29, 2014
 *      Author: dario
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <math.h>
#include <boost/assign.hpp>
#include "particle/MotionModel.h"
#include "particle/Particle.h"
#include "LayoutManager.h"
#include "VisualOdometry.h"

// layout-manager variables
MatrixXd err = MatrixXd::Identity(12,12) * pow(0.05,2.0);
VisualOdometry visual_odometry;
VectorXd p_pose = VectorXd::Zero(12);
MatrixXd p_sigma = MatrixXd::Zero(12,12);
MotionModel mtn_model(err, 1);
Particle particle(99, p_pose, p_sigma, mtn_model);
LayoutManager layout_manager;
ros::Publisher pub;


/**
 * Return a 12x1 VectorXd representing the pose given
 * a message of type nav_msgs::Odometryfrom
 * @param msg
 * @return
 */
VectorXd getPoseFromOdom(const nav_msgs::Odometry& msg);
MatrixXd getCovFromOdom(const nav_msgs::Odometry& msg);

/**
 * Returns a message of type nav_msgs::Odometry given
 * a VectorXd as parameter
 * @param pose
 * @return
 */
nav_msgs::Odometry getOdomFromPoseAndSigma(const VectorXd& pose, const MatrixXd& sigma);

/**
 * Callback called on nav_msg::Odometry arrival
 * @param msg
 */
void odometryCallback(const nav_msgs::Odometry& msg)
{
	// retrieve pose from odometry
	VectorXd p_pose = getPoseFromOdom(msg);
	particle.setParticleState(p_pose);

	// retrieve sigma from odometry
	MatrixXd p_sigma = getCovFromOdom(msg);
	particle.setParticleSigma(p_sigma);

	// call particle_estimation
	layout_manager.particleEstimation(particle);

	// publish the estimated particle odometry
	nav_msgs::Odometry pub_msg = getOdomFromPoseAndSigma(particle.getParticleState(), particle.getParticleSigma());
	pub.publish(pub_msg);

//	ROS_INFO_STREAM("Particle state (after EKF): " << endl << particle.getParticleState() << endl);
//	ROS_INFO_STREAM("Particle sigma (after EKF): " << endl << particle.getParticleSigma() << endl);
}

int main(int argc, char **argv)
{
	// init ROS and NodeHandle
	ros::init(argc, argv, "layout_manager");
	ros::NodeHandle n;

	// init layout-manager variables
	particle.mtn_model.setErrorCovariance(err);
	layout_manager.setVisualOdometry(visual_odometry);
	layout_manager.visual_odometry.setErrorCovariance(err);

	// init subscriber
	ros::Subscriber sub = n.subscribe("visual_odometry/odom_no_error", 1000, odometryCallback);
	pub = n.advertise<nav_msgs::Odometry>("layout_manager/particle_pose",1000);
	ros::spin();

	return 0;
}

VectorXd getPoseFromOdom(const nav_msgs::Odometry& msg)
{
	VectorXd pose = VectorXd::Zero(12);

	//state
	pose(0) = msg.pose.pose.position.x;
	pose(1) = msg.pose.pose.position.y;
	pose(2) = msg.pose.pose.position.z;

	//orientation
	tf::Quaternion q(
			msg.pose.pose.orientation.x,
			msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z,
			msg.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);

	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	pose(3) = roll;
	pose(4) = pitch;
	pose(5) = yaw;

	//speed
	pose(6) = msg.twist.twist.linear.x;
	pose(7) = msg.twist.twist.linear.y;
	pose(8) = msg.twist.twist.linear.z;
	pose(9) = msg.twist.twist.angular.x;
	pose(10) = msg.twist.twist.angular.y;
	pose(11) = msg.twist.twist.angular.z;

	return pose;
}

MatrixXd getCovFromOdom(const nav_msgs::Odometry& msg){
	MatrixXd pose_cov = MatrixXd::Zero(6,6);
	MatrixXd twist_cov = MatrixXd::Zero(6,6);
	MatrixXd cov = MatrixXd::Zero(12,12);

	// build cov array from boost::matrix
	double pose_cov_array [36];
	for(int i=0; i<36; i++){
		pose_cov_array[i] = msg.pose.covariance.elems[i];
	}

	// build cov array from boost::matrix
	double twist_cov_array [36];
	for(int i=0; i<36; i++){
		twist_cov_array[i] = msg.twist.covariance.elems[i];
	}

	// build the two 6x6 cov matrix from arrays
	int counter = 0;
	for(int row=0; row<6; row++)
	{
		for(int column=0; column<6; column++)
		{
			pose_cov(row,column) = pose_cov_array[counter];
			twist_cov(row,column) = twist_cov_array[counter];
			counter++;
		}
	}

	// join the two matrix on top-left and bottom-right corner of the 12x12 cov matrix
	cov.topLeftCorner(6,6) = pose_cov;
	cov.bottomRightCorner(6,6) = twist_cov;

	return cov;
}


nav_msgs::Odometry getOdomFromPoseAndSigma(const VectorXd& pose, const MatrixXd& sigma){

	// istantiate odom
	nav_msgs::Odometry odom;

	//position
	odom.pose.pose.position.x = pose(0);
	odom.pose.pose.position.y = pose(1);
	odom.pose.pose.position.z = pose(2);

	//orientation
	tf::Quaternion odom_quat = tf::createQuaternionFromRPY(double(pose(3)), double(pose(4)), double(pose(5)));
	odom.pose.pose.orientation.x = odom_quat.getX();
	odom.pose.pose.orientation.y = odom_quat.getY();
	odom.pose.pose.orientation.z = odom_quat.getZ();
	odom.pose.pose.orientation.w = odom_quat.getW();

	//speed
	odom.twist.twist.linear.x = pose(6);
	odom.twist.twist.linear.y = pose(7);
	odom.twist.twist.linear.z = pose(8);
	odom.twist.twist.angular.x = pose(9);
	odom.twist.twist.angular.y = pose(10);
	odom.twist.twist.angular.z = pose(11);

	//covariance
	MatrixXd pose_cov = MatrixXd::Zero(6,6);
	MatrixXd twist_cov = MatrixXd::Zero(6,6);
	pose_cov = sigma.topLeftCorner(6,6);
	twist_cov = sigma.bottomRightCorner(6,6);

	odom.pose.covariance = boost::assign::list_of
			(double(pose_cov(0,0)))   (double(pose_cov(0,1)))  (double(pose_cov(0,2)))   (double(pose_cov(0,3)))   (double(pose_cov(0,4)))  (double(pose_cov(0,5)))
			(double(pose_cov(1,0)))   (double(pose_cov(1,1)))  (double(pose_cov(1,2)))   (double(pose_cov(1,3)))   (double(pose_cov(1,4)))  (double(pose_cov(1,5)))
			(double(pose_cov(2,0)))   (double(pose_cov(2,1)))  (double(pose_cov(2,2)))   (double(pose_cov(2,3)))   (double(pose_cov(2,4)))  (double(pose_cov(2,5)))
			(double(pose_cov(3,0)))   (double(pose_cov(3,1)))  (double(pose_cov(3,2)))   (double(pose_cov(3,3)))   (double(pose_cov(3,4)))  (double(pose_cov(3,5)))
			(double(pose_cov(4,0)))   (double(pose_cov(4,1)))  (double(pose_cov(4,2)))   (double(pose_cov(4,3)))   (double(pose_cov(4,4)))  (double(pose_cov(4,5)))
			(double(pose_cov(5,0)))   (double(pose_cov(5,1)))  (double(pose_cov(5,2)))   (double(pose_cov(5,3)))   (double(pose_cov(5,4)))  (double(pose_cov(5,5)));

	odom.twist.covariance = boost::assign::list_of
			(double(twist_cov(0,0)))   (double(twist_cov(0,1)))  (double(twist_cov(0,2)))   (double(twist_cov(0,3)))   (double(twist_cov(0,4)))  (double(twist_cov(0,5)))
			(double(twist_cov(1,0)))   (double(twist_cov(1,1)))  (double(twist_cov(1,2)))   (double(twist_cov(1,3)))   (double(twist_cov(1,4)))  (double(twist_cov(1,5)))
			(double(twist_cov(2,0)))   (double(twist_cov(2,1)))  (double(twist_cov(2,2)))   (double(twist_cov(2,3)))   (double(twist_cov(2,4)))  (double(twist_cov(2,5)))
			(double(twist_cov(3,0)))   (double(twist_cov(3,1)))  (double(twist_cov(3,2)))   (double(twist_cov(3,3)))   (double(twist_cov(3,4)))  (double(twist_cov(3,5)))
			(double(twist_cov(4,0)))   (double(twist_cov(4,1)))  (double(twist_cov(4,2)))   (double(twist_cov(4,3)))   (double(twist_cov(4,4)))  (double(twist_cov(4,5)))
			(double(twist_cov(5,0)))   (double(twist_cov(5,1)))  (double(twist_cov(5,2)))   (double(twist_cov(5,3)))   (double(twist_cov(5,4)))  (double(twist_cov(5,5)));


	return odom;
}


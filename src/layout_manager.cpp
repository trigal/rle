/*
 * layout_manager.cpp
 *
 *  Created on: Jul 29, 2014
 *      Author: dario
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <math.h>
#include <boost/assign.hpp>
#include "particle/MotionModel.h"
#include "particle/Particle.h"
#include "LayoutManager.h"
#include "VisualOdometry.h"
#include "Eigen/Core"
#include "Eigen/Dense"
using namespace Eigen;

// layout-manager variables
MatrixXd err = MatrixXd::Identity(12,12) * (0.05*0.05);
VisualOdometry visual_odometry;
VectorXd p_pose = VectorXd::Zero(12);
MatrixXd p_sigma = MatrixXd::Zero(12,12);
MotionModel mtn_model(err, 1);
Particle particle1(1, p_pose, p_sigma, mtn_model);
Particle particle2(2, p_pose, p_sigma, mtn_model);
Particle particle3(3, p_pose, p_sigma, mtn_model);
Particle particle4(4, p_pose, p_sigma, mtn_model);
LayoutManager layout_manager;
ros::Publisher pub;
ros::Publisher array_pub;

/**
 * Return a 12x1 VectorXd representing the pose given
 * a message of type nav_msgs::Odometryfrom
 * @param msg
 * @return
 */
VectorXd getPoseFromOdom(const nav_msgs::Odometry& msg);
MatrixXd getCovFromOdom(const nav_msgs::Odometry& msg);
geometry_msgs::Pose getPoseFromVector(const VectorXd& msg);

double getNoise(double err);
VectorXd addNoiseToVectorXd(const VectorXd& pose, double position_err, double orientation_err);

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
//measurementCallback
void odometryCallback(const nav_msgs::Odometry& msg)
{
	// retrieve pose from odometry
	VectorXd p_pose = getPoseFromOdom(msg);
    particle1.setParticleState(p_pose);

	// retrieve sigma from odometry
	MatrixXd p_sigma = getCovFromOdom(msg);
    particle1.setParticleSigma(p_sigma);

    // adds noise to the other particles
    particle2.setParticleSigma(p_sigma);
    particle2.setParticleState(addNoiseToVectorXd(p_pose, 0.05, -0.005));

    particle3.setParticleSigma(p_sigma);
    particle3.setParticleState(addNoiseToVectorXd(p_pose, 0.15, 0.005));

    particle4.setParticleSigma(p_sigma);
    particle4.setParticleState(addNoiseToVectorXd(p_pose, 0.01, 0.01));

	// call particle_estimation
    layout_manager.particleEstimation(particle1);
    layout_manager.particleEstimation(particle2);
    layout_manager.particleEstimation(particle3);
    layout_manager.particleEstimation(particle4);

    // --------------------------------------------------------------------------------------
    // publish the estimated particle odometry
    nav_msgs::Odometry pub_msg = getOdomFromPoseAndSigma(particle1.getParticleState(), particle1.getParticleSigma());

    //header
    pub_msg.header.frame_id = "robot_frame";
    pub_msg.header.stamp = ros::Time::now();
    pub_msg.child_frame_id = "odom_frame";

	pub.publish(pub_msg);
    // --------------------------------------------------------------------------------------


    // --------------------------------------------------------------------------------------
    // BUILD POSEARRAY MSG
    geometry_msgs::PoseArray array_msg;
    array_msg.header.stamp = ros::Time::now();
    array_msg.header.frame_id = "robot_frame";

    array_msg.poses.push_back(getPoseFromVector(particle1.getParticleState()));
    array_msg.poses.push_back(getPoseFromVector(particle2.getParticleState()));
    array_msg.poses.push_back(getPoseFromVector(particle3.getParticleState()));
    array_msg.poses.push_back(getPoseFromVector(particle4.getParticleState()));

    array_pub.publish(array_msg);
    // --------------------------------------------------------------------------------------
}

int main(int argc, char **argv)
{
	// init ROS and NodeHandle
	ros::init(argc, argv, "layout_manager");
	ros::NodeHandle n;

	// init layout-manager variables
    particle1.mtn_model.setErrorCovariance(err);
    particle2.mtn_model.setErrorCovariance(err);
    particle3.mtn_model.setErrorCovariance(err);
    particle4.mtn_model.setErrorCovariance(err);

    // add some noise to particle poses
    particle2.setParticleState(addNoiseToVectorXd(particle1.getParticleState(), 0.05, -0.005));
    particle3.setParticleState(addNoiseToVectorXd(particle1.getParticleState(), 0.15, 0.005));
    particle4.setParticleState(addNoiseToVectorXd(particle1.getParticleState(), 0.01, 0.01));

    // init layout manager
	layout_manager.setVisualOdometry(visual_odometry);
	layout_manager.visual_odometry.setErrorCovariance(err);

	// init subscriber
    ros::Subscriber sub = n.subscribe("visual_odometry/odom", 2, odometryCallback);

    // init publishers
    pub = n.advertise<nav_msgs::Odometry>("layout_manager/particle_pose",1000);
    array_pub = n.advertise<geometry_msgs::PoseArray>("layout_manager/particle_pose_array",1000);

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
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, q);
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


/**
 * @param err
 * @return a random number between -err and err
 */
double getNoise(double err){
    double noise = (-err) + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(err - (-err))));
    return noise;
}

/**
 * Adds a random noise to a VectorXd 12x1 representing particle's pose
 * @param pose
 * @param err
 */
VectorXd addNoiseToVectorXd(const VectorXd& pose, double position_err, double orientation_err)
{
    VectorXd vec = pose;
    vec(0) += position_err;
    vec(1) += position_err;
    vec(2) += position_err;
    vec(3) += orientation_err;
    vec(4) += orientation_err;
    vec(5) += orientation_err;

    return vec;
}

geometry_msgs::Pose getPoseFromVector(const VectorXd& msg){
   geometry_msgs::Pose pose;

   //set position
   pose.position.x = msg(0);
   pose.position.y = msg(1);
   pose.position.z = msg(2);

   //set orientation
   tf::Quaternion q = tf::createQuaternionFromRPY(msg(3), msg(4), msg(5));
   tf::quaternionTFToMsg(q, pose.orientation);

   return pose;
}

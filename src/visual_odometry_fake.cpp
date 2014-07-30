/*
 * visual_odometry.cpp
 *
 *  Created on: Jul 29, 2014
 *      Author: dario
 */

// This header defines the standard ROS classes
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/assign.hpp>
#include <vector>
#include <stdlib.h>
#include <ctime>
using std::vector;
using namespace Eigen;

// common variables for steps building -----------------------------------------
double roll;
double pitch;
double yaw;
tf::Quaternion odom_quat;
// ------------------------------------------------------------------------------

// const ------------------------------------------------------------------------
int num_step = 6;			 /// numbers of robot steps
double err = 0.05 * 0.05; 	 /// error covariance 0.05^2
double odom_rate = 1;		 /// odometry fps (Hz)
// ------------------------------------------------------------------------------

// functions --------------------------------------------------------------------
vector<nav_msgs::Odometry> buildSteps();
vector<nav_msgs::Odometry> noiseSteps(const vector<nav_msgs::Odometry> & steps);
float getNoise(float err);
// ------------------------------------------------------------------------------


int main(int argc, char **argv)
{
	// Initialize the ROS system
	ros::init(argc, argv, "visual_odometry");

	// Establish this program as a ROS node
	ros::NodeHandle nh;
    tf::TransformBroadcaster odom_broadcaster;

    // Create publisher object
    ros::Publisher pub1 = nh.advertise<nav_msgs::Odometry>("visual_odometry/odom_no_error",1000);
    ros::Publisher pub2 = nh.advertise<nav_msgs::Odometry>("visual_odometry/odom",1000);

    // 30fps -> 30Hz
    ros::Rate rate(odom_rate);

    // Set current time for msg header
    ros::Time current_time;
    current_time = ros::Time::now();

    // build steps vectors
    vector<nav_msgs::Odometry> odom_no_error = buildSteps();
    vector<nav_msgs::Odometry> odom = noiseSteps(odom_no_error);

    int actual_step = 0;

    while(ros::ok()){

    	// cycle through the interval [1,10]
    	int curr_step = (actual_step % num_step) + 1;

        // publish message on topic "visual_odometry/odom_no_error"
        pub1.publish(odom_no_error[curr_step]);

        // publish message on topic "visual_odometry/odom"
        pub2.publish(odom[curr_step]);

        // write message on ROSOUT
//        ROS_INFO_STREAM("***************************************************");
//        ROS_INFO_STREAM("STEP: "<< curr_step);
//        ROS_INFO_STREAM("Sending odometry:");
//        ROS_INFO_STREAM("state: "<< odom_no_error[curr_step].pose.pose);
//        ROS_INFO_STREAM("speed: " << odom_no_error[curr_step].twist.twist);
//        ROS_INFO_STREAM("***************************************************");

//        ROS_INFO_STREAM("***************************************************");
//        ROS_INFO_STREAM("STEP: "<< curr_step);
//        ROS_INFO_STREAM("Sending odometry:");
//        ROS_INFO_STREAM("state: "<< odom[curr_step].pose.pose);
//        ROS_INFO_STREAM("speed: " << odom[curr_step].twist.twist);
//        ROS_INFO_STREAM("***************************************************");

        // wait until next iteration (30Hz)
        rate.sleep();

        // increase step counter
        actual_step = actual_step + 1;
    }
}


vector<nav_msgs::Odometry> buildSteps()
{
	vector<nav_msgs::Odometry> steps;

	// ------- STEP 1 ------------------------------------------------------------------------
	nav_msgs::Odometry step1;
	//timestamp
	step1.header.stamp = ros::Time::now();
	step1.header.frame_id = "robot_frame";
	step1.child_frame_id = "odom_frame";
	//pose
	step1.pose.pose.position.x = 0;
	step1.pose.pose.position.y = 0;
	step1.pose.pose.position.z = 0;
	step1.pose.covariance =    boost::assign::list_of(err)   (0)  (0)   (0)   (0)  (0)
	                                                   (0) (err)  (0)   (0)   (0)  (0)
	                                                   (0)   (0)  (err) (0)   (0)  (0)
	                                                   (0)   (0)  (0)   (err) (0)  (0)
	                                                   (0)   (0)  (0)   (0)  (err) (0)
	                                                   (0)   (0)  (0)   (0)   (0)  (err);
	//orientation
	roll  = 0;
	pitch = 0;
    yaw	  = M_PI/4; //45° a sx
	odom_quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
	step1.pose.pose.orientation.w = float(odom_quat.getW());
	step1.pose.pose.orientation.x = float(odom_quat.getX());
	step1.pose.pose.orientation.y = float(odom_quat.getY());
	step1.pose.pose.orientation.z = float(odom_quat.getZ());

	//velocity
	step1.twist.twist.linear.x = 0.5;
	step1.twist.twist.linear.y = 0.5;
	step1.twist.twist.linear.z = 0;
	step1.twist.twist.angular.x = 0;
	step1.twist.twist.angular.y = 0;
	step1.twist.twist.angular.z = 0;
	step1.twist.covariance =  boost::assign::list_of(err) (0) (0)  (0)  (0)  (0)
	                                               (0) (err)  (0)  (0)  (0)  (0)
	                                               (0)   (0)  (err) (0)  (0)  (0)
	                                               (0)   (0)   (0) (err) (0)  (0)
	                                               (0)   (0)   (0)  (0) (err) (0)
	                                               (0)   (0)   (0)  (0)  (0)  (err) ;

	steps.push_back(step1);
	// ---------------------------------------------------------------------------------------



	// ------- STEP 2 ------------------------------------------------------------------------
	nav_msgs::Odometry step2;
	//timestamp
	step2.header.stamp = ros::Time::now();
	step2.header.frame_id = "robot_frame";
	step2.child_frame_id = "odom_frame";
	//pose
	step2.pose.pose.position.x = 0.5;
	step2.pose.pose.position.y = 0.5;
	step2.pose.pose.position.z = 0;
	step2.pose.covariance =    boost::assign::list_of(err)   (0)  (0)   (0)   (0)  (0)
	                                                   (0) (err)  (0)   (0)   (0)  (0)
	                                                   (0)   (0)  (err) (0)   (0)  (0)
	                                                   (0)   (0)  (0)   (err) (0)  (0)
	                                                   (0)   (0)  (0)   (0)  (err) (0)
	                                                   (0)   (0)  (0)   (0)   (0)  (err);
	//orientation
	roll  = 0;
	pitch = 0;
	yaw	  = 0;
	odom_quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
	step2.pose.pose.orientation.w = float(odom_quat.getW());
	step2.pose.pose.orientation.x = float(odom_quat.getX());
	step2.pose.pose.orientation.y = float(odom_quat.getY());
	step2.pose.pose.orientation.z = float(odom_quat.getZ());

	//velocity
	step2.twist.twist.linear.x = 1;
	step2.twist.twist.linear.y = 0;
	step2.twist.twist.linear.z = 0;
	step2.twist.twist.angular.x = 0;
	step2.twist.twist.angular.y = 0;
	step2.twist.twist.angular.z = 0;
	step2.twist.covariance =  boost::assign::list_of(err) (0) (0)  (0)  (0)  (0)
	                                               (0) (err)  (0)  (0)  (0)  (0)
	                                               (0)   (0)  (err) (0)  (0)  (0)
	                                               (0)   (0)   (0) (err) (0)  (0)
	                                               (0)   (0)   (0)  (0) (err) (0)
	                                               (0)   (0)   (0)  (0)  (0)  (err) ;

	steps.push_back(step2);
	// ---------------------------------------------------------------------------------------




	// ------- STEP 3 ------------------------------------------------------------------------
	nav_msgs::Odometry step3;
	//timestamp
	step3.header.stamp = ros::Time::now();
	step3.header.frame_id = "robot_frame";
	step3.child_frame_id = "odom_frame";
	//pose
	step3.pose.pose.position.x = 1.5;
	step3.pose.pose.position.y = 0.5;
	step3.pose.pose.position.z = 0;
	step3.pose.covariance =    boost::assign::list_of(err)   (0)  (0)   (0)   (0)  (0)
	                                                   (0) (err)  (0)   (0)   (0)  (0)
	                                                   (0)   (0)  (err) (0)   (0)  (0)
	                                                   (0)   (0)  (0)   (err) (0)  (0)
	                                                   (0)   (0)  (0)   (0)  (err) (0)
	                                                   (0)   (0)  (0)   (0)   (0)  (err);
	//orientation
	roll  = 0;
	pitch = 0;
	yaw	  = (-3) * M_PI/4;
	odom_quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
	step3.pose.pose.orientation.w = float(odom_quat.getW());
	step3.pose.pose.orientation.x = float(odom_quat.getX());
	step3.pose.pose.orientation.y = float(odom_quat.getY());
	step3.pose.pose.orientation.z = float(odom_quat.getZ());

	//velocity
	step3.twist.twist.linear.x = 0.5;
	step3.twist.twist.linear.y = -0.5;
	step3.twist.twist.linear.z = 0;
	step3.twist.twist.angular.x = 0;
	step3.twist.twist.angular.y = 0;
	step3.twist.twist.angular.z = 0;
	step3.twist.covariance =  boost::assign::list_of(err) (0) (0)  (0)  (0)  (0)
	                                               (0) (err)  (0)  (0)  (0)  (0)
	                                               (0)   (0)  (err) (0)  (0)  (0)
	                                               (0)   (0)   (0) (err) (0)  (0)
	                                               (0)   (0)   (0)  (0) (err) (0)
	                                               (0)   (0)   (0)  (0)  (0)  (err) ;

	steps.push_back(step3);
	// ---------------------------------------------------------------------------------------





	// ------- STEP 4 ------------------------------------------------------------------------
	nav_msgs::Odometry step4;
	//timestamp
	step4.header.stamp = ros::Time::now();
	step4.header.frame_id = "robot_frame";
	step4.child_frame_id = "odom_frame";
	//pose
	step4.pose.pose.position.x = 2;
	step4.pose.pose.position.y = 0;
	step4.pose.pose.position.z = 0;
	step4.pose.covariance =    boost::assign::list_of(err)   (0)  (0)   (0)   (0)  (0)
	                                                   (0) (err)  (0)   (0)   (0)  (0)
	                                                   (0)   (0)  (err) (0)   (0)  (0)
	                                                   (0)   (0)  (0)   (err) (0)  (0)
	                                                   (0)   (0)  (0)   (0)  (err) (0)
	                                                   (0)   (0)  (0)   (0)   (0)  (err);
	//orientation
	roll  = 0;
	pitch = 0;
    yaw	  = -3 * (M_PI/4);
	odom_quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
	step4.pose.pose.orientation.w = float(odom_quat.getW());
	step4.pose.pose.orientation.x = float(odom_quat.getX());
	step4.pose.pose.orientation.y = float(odom_quat.getY());
	step4.pose.pose.orientation.z = float(odom_quat.getZ());

	//velocity
	step4.twist.twist.linear.x = -0.5;
	step4.twist.twist.linear.y = -0.5;
	step4.twist.twist.linear.z = 0;
	step4.twist.twist.angular.x = 0;
	step4.twist.twist.angular.y = 0;
	step4.twist.twist.angular.z = 0;
	step4.twist.covariance =  boost::assign::list_of(err) (0) (0)  (0)  (0)  (0)
	                                               (0) (err)  (0)  (0)  (0)  (0)
	                                               (0)   (0)  (err) (0)  (0)  (0)
	                                               (0)   (0)   (0) (err) (0)  (0)
	                                               (0)   (0)   (0)  (0) (err) (0)
	                                               (0)   (0)   (0)  (0)  (0)  (err) ;

	steps.push_back(step4);
	// ---------------------------------------------------------------------------------------



	// ------- STEP 5 ------------------------------------------------------------------------
	nav_msgs::Odometry step5;
	//timestamp
	step5.header.stamp = ros::Time::now();
	step5.header.frame_id = "robot_frame";
	step5.child_frame_id = "odom_frame";
	//pose
	step5.pose.pose.position.x = 1.5;
	step5.pose.pose.position.y = -0.5;
	step5.pose.pose.position.z = 0;
	step5.pose.covariance =    boost::assign::list_of(err)   (0)  (0)   (0)   (0)  (0)
	                                                   (0) (err)  (0)   (0)   (0)  (0)
	                                                   (0)   (0)  (err) (0)   (0)  (0)
	                                                   (0)   (0)  (0)   (err) (0)  (0)
	                                                   (0)   (0)  (0)   (0)  (err) (0)
	                                                   (0)   (0)  (0)   (0)   (0)  (err);
	//orientation
	roll  = 0;
	pitch = 0;
    yaw	  = M_PI;
	odom_quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
	step5.pose.pose.orientation.w = float(odom_quat.getW());
	step5.pose.pose.orientation.x = float(odom_quat.getX());
	step5.pose.pose.orientation.y = float(odom_quat.getY());
	step5.pose.pose.orientation.z = float(odom_quat.getZ());

	//velocity
	step5.twist.twist.linear.x = -1;
	step5.twist.twist.linear.y = 0;
	step5.twist.twist.linear.z = 0;
	step5.twist.twist.angular.x = 0;
	step5.twist.twist.angular.y = 0;
	step5.twist.twist.angular.z = 0;
	step5.twist.covariance =  boost::assign::list_of(err) (0) (0)  (0)  (0)  (0)
	                                               (0) (err)  (0)  (0)  (0)  (0)
	                                               (0)   (0)  (err) (0)  (0)  (0)
	                                               (0)   (0)   (0) (err) (0)  (0)
	                                               (0)   (0)   (0)  (0) (err) (0)
	                                               (0)   (0)   (0)  (0)  (0)  (err) ;

	steps.push_back(step5);
	// ---------------------------------------------------------------------------------------



	// ------- STEP 6 ------------------------------------------------------------------------
	nav_msgs::Odometry step6;
	//timestamp
	step6.header.stamp = ros::Time::now();
	step6.header.frame_id = "robot_frame";
	step6.child_frame_id = "odom_frame";
	//pose
	step6.pose.pose.position.x = 0.5;
	step6.pose.pose.position.y = -0.5;
	step6.pose.pose.position.z = 0;
	step6.pose.covariance =    boost::assign::list_of(err)   (0)  (0)   (0)   (0)  (0)
	                                                   (0) (err)  (0)   (0)   (0)  (0)
	                                                   (0)   (0)  (err) (0)   (0)  (0)
	                                                   (0)   (0)  (0)   (err) (0)  (0)
	                                                   (0)   (0)  (0)   (0)  (err) (0)
	                                                   (0)   (0)  (0)   (0)   (0)  (err);
	//orientation
	roll  = 0;
	pitch = 0;
    yaw	  = 3 * (M_PI/4);
	odom_quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
	step6.pose.pose.orientation.w = float(odom_quat.getW());
	step6.pose.pose.orientation.x = float(odom_quat.getX());
	step6.pose.pose.orientation.y = float(odom_quat.getY());
	step6.pose.pose.orientation.z = float(odom_quat.getZ());

	//velocity
	step6.twist.twist.linear.x = -0.5;
	step6.twist.twist.linear.y = 0.5;
	step6.twist.twist.linear.z = 0;
	step6.twist.twist.angular.x = 0;
	step6.twist.twist.angular.y = 0;
	step6.twist.twist.angular.z = 0;
	step6.twist.covariance =  boost::assign::list_of(err) (0) (0)  (0)  (0)  (0)
	                                               (0) (err)  (0)  (0)  (0)  (0)
	                                               (0)   (0)  (err) (0)  (0)  (0)
	                                               (0)   (0)   (0) (err) (0)  (0)
	                                               (0)   (0)   (0)  (0) (err) (0)
	                                               (0)   (0)   (0)  (0)  (0)  (err) ;

	steps.push_back(step6);
	// ---------------------------------------------------------------------------------------



	// ------- STEP 7 ------------------------------------------------------------------------
	nav_msgs::Odometry step7;
	//timestamp
	step7.header.stamp = ros::Time::now();
	//pose
	step7.pose.pose.position.x = 0;
	step7.pose.pose.position.y = 0;
	step7.pose.pose.position.z = 0;
	step7.pose.covariance =    boost::assign::list_of(err)   (0)  (0)   (0)   (0)  (0)
	                                                   (0) (err)  (0)   (0)   (0)  (0)
	                                                   (0)   (0)  (err) (0)   (0)  (0)
	                                                   (0)   (0)  (0)   (err) (0)  (0)
	                                                   (0)   (0)  (0)   (0)  (err) (0)
	                                                   (0)   (0)  (0)   (0)   (0)  (err);
	//orientation
	roll  = 0.0523;
	pitch = 0.0523;
	yaw	  = 0.0523;
	odom_quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
	step7.pose.pose.orientation.w = float(odom_quat.getW());
	step7.pose.pose.orientation.x = float(odom_quat.getX());
	step7.pose.pose.orientation.y = float(odom_quat.getY());
	step7.pose.pose.orientation.z = float(odom_quat.getZ());

	//velocity
	step7.twist.twist.linear.x = 0;
	step7.twist.twist.linear.y = 0;
	step7.twist.twist.linear.z = 0;
	step7.twist.twist.angular.x = 0;
	step7.twist.twist.angular.y = 0;
	step7.twist.twist.angular.z = 0;
	step7.twist.covariance =  boost::assign::list_of(err) (0) (0)  (0)  (0)  (0)
	                                               (0) (err)  (0)  (0)  (0)  (0)
	                                               (0)   (0)  (err) (0)  (0)  (0)
	                                               (0)   (0)   (0) (err) (0)  (0)
	                                               (0)   (0)   (0)  (0) (err) (0)
	                                               (0)   (0)   (0)  (0)  (0)  (err) ;

	steps.push_back(step7);
	// ---------------------------------------------------------------------------------------



	// ------- STEP 8 ------------------------------------------------------------------------
	nav_msgs::Odometry step8;
	//timestamp
	step8.header.stamp = ros::Time::now();
	//pose
	step8.pose.pose.position.x = 0;
	step8.pose.pose.position.y = 0;
	step8.pose.pose.position.z = 0;
	step8.pose.covariance =    boost::assign::list_of(err)   (0)  (0)   (0)   (0)  (0)
	                                                   (0) (err)  (0)   (0)   (0)  (0)
	                                                   (0)   (0)  (err) (0)   (0)  (0)
	                                                   (0)   (0)  (0)   (err) (0)  (0)
	                                                   (0)   (0)  (0)   (0)  (err) (0)
	                                                   (0)   (0)  (0)   (0)   (0)  (err);
	//orientation
	roll  = 0.0523;
	pitch = 0.0523;
	yaw	  = 0.0523;
	odom_quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
	step8.pose.pose.orientation.w = float(odom_quat.getW());
	step8.pose.pose.orientation.x = float(odom_quat.getX());
	step8.pose.pose.orientation.y = float(odom_quat.getY());
	step8.pose.pose.orientation.z = float(odom_quat.getZ());

	//velocity
	step8.twist.twist.linear.x = 0;
	step8.twist.twist.linear.y = 0;
	step8.twist.twist.linear.z = 0;
	step8.twist.twist.angular.x = 0;
	step8.twist.twist.angular.y = 0;
	step8.twist.twist.angular.z = 0;
	step8.twist.covariance =  boost::assign::list_of(err) (0) (0)  (0)  (0)  (0)
	                                               (0) (err)  (0)  (0)  (0)  (0)
	                                               (0)   (0)  (err) (0)  (0)  (0)
	                                               (0)   (0)   (0) (err) (0)  (0)
	                                               (0)   (0)   (0)  (0) (err) (0)
	                                               (0)   (0)   (0)  (0)  (0)  (err) ;

	steps.push_back(step8);
	// ---------------------------------------------------------------------------------------



	// ------- STEP 9 ------------------------------------------------------------------------
	nav_msgs::Odometry step9;
	//timestamp
	step9.header.stamp = ros::Time::now();
	//pose
	step9.pose.pose.position.x = 0;
	step9.pose.pose.position.y = 0;
	step9.pose.pose.position.z = 0;
	step9.pose.covariance =    boost::assign::list_of(err)   (0)  (0)   (0)   (0)  (0)
	                                                   (0) (err)  (0)   (0)   (0)  (0)
	                                                   (0)   (0)  (err) (0)   (0)  (0)
	                                                   (0)   (0)  (0)   (err) (0)  (0)
	                                                   (0)   (0)  (0)   (0)  (err) (0)
	                                                   (0)   (0)  (0)   (0)   (0)  (err);
	//orientation
	roll  = 0.0523;
	pitch = 0.0523;
	yaw	  = 0.0523;
	odom_quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
	step9.pose.pose.orientation.w = float(odom_quat.getW());
	step9.pose.pose.orientation.x = float(odom_quat.getX());
	step9.pose.pose.orientation.y = float(odom_quat.getY());
	step9.pose.pose.orientation.z = float(odom_quat.getZ());

	//velocity
	step9.twist.twist.linear.x = 0;
	step9.twist.twist.linear.y = 0;
	step9.twist.twist.linear.z = 0;
	step9.twist.twist.angular.x = 0;
	step9.twist.twist.angular.y = 0;
	step9.twist.twist.angular.z = 0;
	step9.twist.covariance =  boost::assign::list_of(err) (0) (0)  (0)  (0)  (0)
	                                               (0) (err)  (0)  (0)  (0)  (0)
	                                               (0)   (0)  (err) (0)  (0)  (0)
	                                               (0)   (0)   (0) (err) (0)  (0)
	                                               (0)   (0)   (0)  (0) (err) (0)
	                                               (0)   (0)   (0)  (0)  (0)  (err) ;

	steps.push_back(step9);
	// ---------------------------------------------------------------------------------------



	// ------- STEP 10 -----------------------------------------------------------------------
	nav_msgs::Odometry step10;
	//timestamp
	step10.header.stamp = ros::Time::now();
	//pose
	step10.pose.pose.position.x = 0;
	step10.pose.pose.position.y = 0;
	step10.pose.pose.position.z = 0;
	step10.pose.covariance =    boost::assign::list_of(err)   (0)  (0)   (0)   (0)  (0)
	                                                   (0) (err)  (0)   (0)   (0)  (0)
	                                                   (0)   (0)  (err) (0)   (0)  (0)
	                                                   (0)   (0)  (0)   (err) (0)  (0)
	                                                   (0)   (0)  (0)   (0)  (err) (0)
	                                                   (0)   (0)  (0)   (0)   (0)  (err);
	//orientation
	roll  = 0.0523;
	pitch = 0.0523;
	yaw	  = 0.0523;
	odom_quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
	step10.pose.pose.orientation.w = float(odom_quat.getW());
	step10.pose.pose.orientation.x = float(odom_quat.getX());
	step10.pose.pose.orientation.y = float(odom_quat.getY());
	step10.pose.pose.orientation.z = float(odom_quat.getZ());

	//velocity
	step10.twist.twist.linear.x = 0;
	step10.twist.twist.linear.y = 0;
	step10.twist.twist.linear.z = 0;
	step10.twist.twist.angular.x = 0;
	step10.twist.twist.angular.y = 0;
	step10.twist.twist.angular.z = 0;
	step10.twist.covariance =  boost::assign::list_of(err) (0) (0)  (0)  (0)  (0)
	                                               (0) (err)  (0)  (0)  (0)  (0)
	                                               (0)   (0)  (err) (0)  (0)  (0)
	                                               (0)   (0)   (0) (err) (0)  (0)
	                                               (0)   (0)   (0)  (0) (err) (0)
	                                               (0)   (0)   (0)  (0)  (0)  (err) ;

	steps.push_back(step10);
	// ---------------------------------------------------------------------------------------

	return steps;
}


/**
 * @param err
 * @return a random number between -err and err
 */
float getNoise(float err){
	float noise = (-err) + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(err - (-err))));
	return noise;
}

/**
 * Adds a random noise to odometry steps
 * @param steps
 * @return noisy steps
 */
vector<nav_msgs::Odometry> noiseSteps(const vector<nav_msgs::Odometry> & steps)
{
	// rand seed
	srand(time(0));

	// initialize noisy_steps vector
	vector<nav_msgs::Odometry> noisy_steps;

	for(int i=0; i<num_step; i++)
	{
		// adds noise to the step
		nav_msgs::Odometry step = steps.at(i);
		step.header.frame_id = "robot_frame";
		step.child_frame_id = "odom_frame";
		step.pose.pose.position.x += getNoise(step.pose.covariance.elems[0]);
		step.pose.pose.position.y += getNoise(step.pose.covariance.elems[7]);
		step.pose.pose.position.z += getNoise(step.pose.covariance.elems[14]);
//		step.pose.pose.orientation.w += getNoise(step.pose.covariance.elems[21]);
		step.pose.pose.orientation.x += getNoise(step.pose.covariance.elems[21]);
		step.pose.pose.orientation.y += getNoise(step.pose.covariance.elems[28]);
		step.pose.pose.orientation.z += getNoise(step.pose.covariance.elems[35]);

		step.twist.twist.angular.x += getNoise(step.twist.covariance.elems[0]);
		step.twist.twist.angular.y += getNoise(step.twist.covariance.elems[7]);
		step.twist.twist.angular.z += getNoise(step.twist.covariance.elems[14]);
		step.twist.twist.linear.x += getNoise(step.twist.covariance.elems[21]);
		step.twist.twist.linear.y += getNoise(step.twist.covariance.elems[28]);
		step.twist.twist.linear.z += getNoise(step.twist.covariance.elems[35]);

		// push step inside noisy_steps vector
		noisy_steps.push_back(step);
	}
	return noisy_steps;
}

/*
 * visual_odometry.cpp
 *
 *  Created on: Jul 29, 2014
 *      Author: dario
 */

#include "Utils.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
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
using namespace std;
using namespace ros;

// common variables for steps building ***********************************************************
tf::TransformBroadcaster* tfb_;
tf::TransformListener* tf_;
// ***********************************************************************************************

// const *****************************************************************************************
double odom_err = 0.05*0.05; /// measure error covariance (uncertainty^2)
double odom_rate = 30;		 /// odometry fps (Hz)
// ***********************************************************************************************

// functions *************************************************************************************
nav_msgs::Odometry addNoiseToOdom(const nav_msgs::Odometry & step);
geometry_msgs::Twist getSpeed(const double & rate, const tf::Transform & temp_t, const tf::Transform & t);
// ***********************************************************************************************

/**
 *************************************************************************************************
 * @brief main
 *************************************************************************************************
 */
int main(int argc, char **argv)
{
    ROS_INFO_STREAM("VISUAL ODOMETRY FAKE STARTED");

	// Initialize the ROS system
	ros::init(argc, argv, "visual_odometry");

	// Establish this program as a ROS node
    ros::NodeHandle nh;

    // Create publisher object
    ros::Publisher pub1 = nh.advertise<nav_msgs::Odometry>("visual_odometry/odom_no_error",1);
    ros::Publisher pub2 = nh.advertise<nav_msgs::Odometry>("visual_odometry/odom",1);
    ros::Publisher pub3 = nh.advertise<geometry_msgs::PoseArray>("visual_odometry/single_pose_array",1);
    ros::Publisher pub4 = nh.advertise<geometry_msgs::PoseArray>("visual_odometry/single_pose_array_no_err",1);

    // 30fps -> 30Hz
    ros::Rate rate(odom_rate);

    // Set current time for msg header
    ros::Time current_time;
    current_time = ros::Time::now();

    // tf variables
    tfb_ = new tf::TransformBroadcaster();
    tf_ = new tf::TransformListener();

    // rotation set-up
    tf::Transform temp_t; //used for getting particle speed
    tf::Transform t(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)); //WORLD
    tf::Pose a,b,c;
    a.setOrigin(tf::Vector3(0,0,0)); a.setRotation(tf::createIdentityQuaternion());
    b.setOrigin(tf::Vector3(1,0,0)); b.setRotation(tf::createQuaternionFromYaw(90.0f*3.14f/180.0f));
    c.setOrigin(tf::Vector3(0.1,0,0)); c.setRotation(tf::createQuaternionFromYaw(5.0f*3.14f/180.0f));
    t=a.inverseTimes(b);
    //tfb_->sendTransform(tf::StampedTransform(t, ros::Time::now(), "robot_frame", "odom_frame"));
    t.setRotation(t.getRotation().normalized());


    ros::Duration(2).sleep(); // sleep for two seconds, system startup

    while(ros::ok()){
        current_time = ros::Time::now();

        // rotate (before rotation, save T into a temp variable)
        temp_t = t;
        t=t*c;
        t.setRotation(t.getRotation().normalized());
        tfb_->sendTransform(tf::StampedTransform(t, current_time, "robot_frame", "odom_frame"));

        // build msg
        nav_msgs::Odometry msg;

        // set header
        msg.header.stamp = current_time;
        msg.header.frame_id = "robot_frame";
        msg.child_frame_id = "odom_frame";

        // set position
        msg.pose.pose.position.x = t.getOrigin().getX();
        msg.pose.pose.position.y = t.getOrigin().getY();
        msg.pose.pose.position.z = t.getOrigin().getZ();

        // set orientation
        tf::quaternionTFToMsg(t.getRotation().normalized(), msg.pose.pose.orientation);

        // set speed
        msg.twist.twist = getSpeed( 1/odom_rate, temp_t, t);

        // publish message on topic "visual_odometry/odom_no_error"
        pub1.publish(msg);

        // publish message on topic "visual_odometry/odom"
        nav_msgs::Odometry noisy_msg = addNoiseToOdom(msg);
        pub2.publish(noisy_msg);

        // publish single posearray with no error on topic "visual_odometry/single_pose_array_no_err"
        geometry_msgs::PoseArray array;
        geometry_msgs::Pose p;
        p = msg.pose.pose;
        array.poses.push_back(p);
        array.header.stamp = current_time;
        array.header.frame_id = "robot_frame";

        pub4.publish(array);

        // publish single posearray with error on topic "visual_odometry/single_pose_array"
        geometry_msgs::PoseArray p_array;
        geometry_msgs::Pose p_pose;
        p_pose = noisy_msg.pose.pose;
        p_array.poses.push_back(p_pose);
        p_array.header.stamp = current_time;
        p_array.header.frame_id = "robot_frame";

        pub3.publish(p_array);


        ros::spinOnce();

        // write message on ROSOUT
//        ROS_INFO_STREAM("***************************************************");
//        ROS_INFO_STREAM("Sending odometry:");
//        ROS_INFO_STREAM("state: "<< tf::getYaw((msg.pose.pose.orientation)));
//        ROS_INFO_STREAM("speed: " << msg.twist.twist);
//        ROS_INFO_STREAM("***************************************************");

//        ROS_INFO_STREAM("***************************************************");
//        ROS_INFO_STREAM("Sending odometry (+ noise):");
//        ROS_INFO_STREAM("state: "<< msg.pose.pose);
//        ROS_INFO_STREAM("speed: " << msg.twist.twist);
//        ROS_INFO_STREAM("***************************************************");

        // wait until next iteration
        rate.sleep();
    }
}






/** **********************************************************************************************
/* FUNCTIONS IMPLEMENTATIONS
/************************************************************************************************/
/**
 * Adds a random noise to odometry SINGLE step
 * @param steps
 * @return noisy steps
 */
nav_msgs::Odometry addNoiseToOdom(const nav_msgs::Odometry & step)
{

    nav_msgs::Odometry noisy_step = step;
    //set covariance
    noisy_step.twist.covariance =  boost::assign::list_of  (odom_err) (0)   (0)  (0)  (0)  (0)
                                                           (0)  (odom_err)  (0)  (0)  (0)  (0)
                                                           (0)   (0)  (odom_err) (0)  (0)  (0)
                                                           (0)   (0)   (0) (odom_err) (0)  (0)
                                                           (0)   (0)   (0)  (0) (odom_err) (0)
                                                           (0)   (0)   (0)  (0)  (0)  (odom_err) ;

    noisy_step.pose.covariance =  boost::assign::list_of  (odom_err) (0)  ( 0)  (0)  (0)  (0)
                                                          (0) (odom_err)   (0)  (0)  (0)  (0)
                                                          (0)   (0)  (odom_err) (0)  (0)  (0)
                                                          (0)   (0)   (0) (odom_err) (0)  (0)
                                                          (0)   (0)   (0)  (0) (odom_err) (0);
    // adds noise to the step
    noisy_step.pose.pose.position.x += Utils::getNoise(noisy_step.pose.covariance.elems[0]);
    noisy_step.pose.pose.position.y += Utils::getNoise(noisy_step.pose.covariance.elems[7]);
    noisy_step.pose.pose.position.z += Utils::getNoise(noisy_step.pose.covariance.elems[14]);
    noisy_step.pose.pose.orientation.x += Utils::getNoise(noisy_step.pose.covariance.elems[21]);
    noisy_step.pose.pose.orientation.y += Utils::getNoise(noisy_step.pose.covariance.elems[28]);
    noisy_step.pose.pose.orientation.z += Utils::getNoise(noisy_step.pose.covariance.elems[35]);

    //normalize angles
    noisy_step.pose.pose.orientation.x = Utils::normalize_angle(noisy_step.pose.pose.orientation.x);
    noisy_step.pose.pose.orientation.y = Utils::normalize_angle(noisy_step.pose.pose.orientation.y);
    noisy_step.pose.pose.orientation.z = Utils::normalize_angle(noisy_step.pose.pose.orientation.z);

    noisy_step.twist.twist.angular.x += Utils::getNoise(noisy_step.twist.covariance.elems[0]);
    noisy_step.twist.twist.angular.y += Utils::getNoise(noisy_step.twist.covariance.elems[7]);
    noisy_step.twist.twist.angular.z += Utils::getNoise(noisy_step.twist.covariance.elems[14]);
    noisy_step.twist.twist.linear.x += Utils::getNoise(noisy_step.twist.covariance.elems[21]);
    noisy_step.twist.twist.linear.y += Utils::getNoise(noisy_step.twist.covariance.elems[28]);
    noisy_step.twist.twist.linear.z += Utils::getNoise(noisy_step.twist.covariance.elems[35]);

    return noisy_step;
}

geometry_msgs::Twist getSpeed(const double & rate, const tf::Transform & temp_t, const tf::Transform & t){

    geometry_msgs::Twist speed;

    speed.linear.x = (t.getOrigin().getX() - temp_t.getOrigin().getX()) / rate;
    speed.linear.y = (t.getOrigin().getY() - temp_t.getOrigin().getY()) / rate;
    speed.linear.z = (t.getOrigin().getZ() - temp_t.getOrigin().getZ()) / rate;

    // Quaternion to RPY (step_prec)
    tf::Matrix3x3 m(temp_t.getRotation());
    double roll_prec; double pitch_prec; double yaw_prec;
    m.getRPY(roll_prec, pitch_prec, yaw_prec);

    // Quaternion to RPY (step_t)
    tf::Matrix3x3 m_t(t.getRotation());
    double roll_t; double pitch_t; double yaw_t;
    m_t.getRPY(roll_t, pitch_t, yaw_t);

    speed.angular.x = ( Utils::angle_diff(roll_t, roll_prec) ) / rate;
    speed.angular.y = ( Utils::angle_diff(pitch_t, pitch_prec) ) / rate;
    speed.angular.z = ( Utils::angle_diff(yaw_t, yaw_prec) ) / rate;

    return speed;
}

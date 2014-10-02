/*
 * visual_odometry_test.cpp
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

// vars -----------------------------------------------------------------------------------------
unsigned int odom_rate = 1; /// rate of msgs published by this node
double movement_rate = 0.1; /// rate of movement between each step

ros::Time current_time;

// functions ------------------------------------------------------------------------------------
nav_msgs::Odometry getXMsg();
nav_msgs::Odometry getYMsg();
nav_msgs::Odometry getZMsg();

/**
 *************************************************************************************************
 * @brief main
 *************************************************************************************************
 */
int main(int argc, char **argv)
{
    ROS_INFO_STREAM("VISUAL ODOMETRY TEST STARTED");

    // Initialize the ROS system
    ros::init(argc, argv, "visual_odometry_test");

    // Establish this program as a ROS node
    ros::NodeHandle nh;

    // Create publisher object
    ros::Publisher pub_x = nh.advertise<nav_msgs::Odometry>("visual_odom_test/axis_x",1);
    ros::Publisher pub_y = nh.advertise<nav_msgs::Odometry>("visual_odom_test/axis_y",1);
    ros::Publisher pub_z = nh.advertise<nav_msgs::Odometry>("visual_odom_test/axis_z",1);
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("visual_odom_test/test",1);

    // 30fps -> 30Hz
    ros::Rate rate(odom_rate);

    // --------- Publish first msg ------------------------------
    unsigned int i = 0;
    // build axis message
    current_time = ros::Time::now();
    nav_msgs::Odometry msg_x = getXMsg();
    nav_msgs::Odometry msg_y = getYMsg();
    nav_msgs::Odometry msg_z = getZMsg();

    nav_msgs::Odometry msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = "robot_frame";
    msg.child_frame_id = "odom_frame";

    msg.pose.pose.position.x = 0;
    msg.pose.pose.position.y = 0;
    msg.pose.pose.position.z = 0;
    tf::Quaternion q = tf::createQuaternionFromYaw(0);
    tf::quaternionTFToMsg(q, msg.pose.pose.orientation);
    msg.twist.twist.linear.x = movement_rate / odom_rate;

    pub.publish(msg);
    rate.sleep();
    // ----------------------------------------------------------

    while(ros::ok()){
        // Set current time for msg header
        current_time = ros::Time::now();

        // move on X axis by 1
        msg.pose.pose.position.x += movement_rate;

        std::cout << "--------------------------------------------------------------------------------" << endl;
        std::cout << "[ Sent msg " << i << "]:" << std::endl;
        std::cout << " Position:" << std::endl;
        std::cout << "  x: " << msg.pose.pose.position.x << std::endl;
        std::cout << "  y: " << msg.pose.pose.position.y << std::endl;
        std::cout << "  z: " << msg.pose.pose.position.z << std::endl;
        std::cout << " Orientation quaternion: " << std::endl;
        std::cout << "  w: " << msg.pose.pose.orientation.w << std::endl;
        std::cout << "  x: " << msg.pose.pose.orientation.x << std::endl;
        std::cout << "  y: " << msg.pose.pose.orientation.y << std::endl;
        std::cout << "  z: " << msg.pose.pose.orientation.z << std::endl;
        std::cout << " Linear speed: " << std::endl;
        std::cout << "  x: " << msg.twist.twist.linear.x << std::endl;
        std::cout << "  y: " << msg.twist.twist.linear.y << std::endl;
        std::cout << "  z: " << msg.twist.twist.linear.z << std::endl;
        std::cout << " Angular speed: " << std::endl;
        std::cout << "  x: " << msg.twist.twist.angular.x << std::endl;
        std::cout << "  y: " << msg.twist.twist.angular.y << std::endl;
        std::cout << "  z: " << msg.twist.twist.angular.z << std::endl;
        std::cout << std::endl;

        // publish robot odom
        pub.publish(msg);

        // publish axis msgs
        pub_x.publish(msg_x);
        pub_y.publish(msg_y);
        pub_z.publish(msg_z);

        // spin
        ros::spinOnce();

        // wait until next iteration
        rate.sleep();
        i++;
    }
}




/** **********************************************************************************************
/* FUNCTIONS IMPLEMENTATIONS
/************************************************************************************************/

nav_msgs::Odometry getXMsg(){
    nav_msgs::Odometry msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = "robot_frame";
    msg.child_frame_id = "odom_frame";

    msg.pose.pose.position.x = 0;
    msg.pose.pose.position.y = 0;
    msg.pose.pose.position.z = 0;

    tf::Quaternion q = tf::createQuaternionFromYaw(0);
    tf::quaternionTFToMsg(q, msg.pose.pose.orientation);

    return msg;
}

nav_msgs::Odometry getYMsg(){
    nav_msgs::Odometry msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = "robot_frame";
    msg.child_frame_id = "odom_frame";

    msg.pose.pose.position.x = 0;
    msg.pose.pose.position.y = 0;
    msg.pose.pose.position.z = 0;

    tf::Quaternion q = tf::createQuaternionFromYaw(M_PI/2);
    tf::quaternionTFToMsg(q, msg.pose.pose.orientation);

    return msg;
}

nav_msgs::Odometry getZMsg(){
    nav_msgs::Odometry msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = "robot_frame";
    msg.child_frame_id = "odom_frame";

    msg.pose.pose.position.x = 0;
    msg.pose.pose.position.y = 0;
    msg.pose.pose.position.z = 0;

    tf::Quaternion q = tf::createQuaternionFromRPY(0, -M_PI/2, 0);
    tf::quaternionTFToMsg(q, msg.pose.pose.orientation);

    return msg;
}






















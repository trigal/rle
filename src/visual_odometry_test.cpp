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
unsigned int publish_rate = 10;  /// rate of msgs published by this node
double movement_rate = 0.1;     /// rate of movement between each step
double msr_cov = 0.05*0.05;     /// msr cov.

ros::Time current_time;
tf::TransformBroadcaster* tfb_;
tf::TransformListener* tf_;
nav_msgs::Odometry msg;

// functions ------------------------------------------------------------------------------------
nav_msgs::Odometry getXMsg();
nav_msgs::Odometry getYMsg();
nav_msgs::Odometry getZMsg();
nav_msgs::Odometry createOdomMsgFromTF(tf::Transform& t);

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

    // Node publish rate
    ros::Rate rate(publish_rate);

    // --------- Publish first msg ------------------------------
    unsigned int i = 0;
    // build axis message
    current_time = ros::Time::now();
    nav_msgs::Odometry msg_x = getXMsg();
    nav_msgs::Odometry msg_y = getYMsg();
    nav_msgs::Odometry msg_z = getZMsg();

    // tf variables
    tfb_ = new tf::TransformBroadcaster();
    tf_ = new tf::TransformListener();
    tf::Transform t(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)); //WORLD

    // move on x axis with movement_rate transform
    tf::Transform move_x(tf::createIdentityQuaternion(), tf::Vector3(movement_rate,0,0));

    // send transform_msg
    tfb_->sendTransform(tf::StampedTransform(t, current_time, "robot_frame", "odom_frame"));

    // init & send odom_msg
    msg = createOdomMsgFromTF(t);
    pub.publish(msg);

    rate.sleep();
    // ----------------------------------------------------------

    while(ros::ok()){

        // apply transform
        t = t * move_x;

        // update current time
        current_time = ros::Time::now();

        // create msg from transform
        msg = createOdomMsgFromTF(t);
        tfb_->sendTransform(tf::StampedTransform(t, current_time, "robot_frame", "odom_frame"));

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

    tf::Quaternion q1 = tf::createQuaternionFromYaw(0);
    tf::Quaternion q = tf::createQuaternionFromRPY(0, -M_PI/2, 0);


    tf::quaternionTFToMsg(q1*q, msg.pose.pose.orientation);

    return msg;
}

nav_msgs::Odometry createOdomMsgFromTF(tf::Transform& t)
{
    nav_msgs::Odometry msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = "robot_frame";
    msg.child_frame_id = "odom_frame";

    // set initial position
    msg.pose.pose.position.x = t.getOrigin().getX();
    msg.pose.pose.position.y = t.getOrigin().getY();
    msg.pose.pose.position.z = t.getOrigin().getZ();

    // set orientation
    t.setRotation(t.getRotation().normalized());
    tf::quaternionTFToMsg(t.getRotation(), msg.pose.pose.orientation);

    // set speed (it will be constant for all the execution)
    msg.twist.twist.linear.x = movement_rate / publish_rate;

    // set covs (they will be constant for all the execution)
    msg.twist.covariance =  boost::assign::list_of  (msr_cov) (0)   (0)  (0)  (0)  (0)
                                                       (0)  (msr_cov)  (0)  (0)  (0)  (0)
                                                       (0)   (0)  (msr_cov) (0)  (0)  (0)
                                                       (0)   (0)   (0) (msr_cov) (0)  (0)
                                                       (0)   (0)   (0)  (0) (msr_cov) (0)
                                                       (0)   (0)   (0)  (0)  (0)  (msr_cov) ;

    msg.pose.covariance =  boost::assign::list_of  (msr_cov) (0)  (0)  (0)  (0)  (0)
                                                      (0) (msr_cov)   (0)  (0)  (0)  (0)
                                                      (0)   (0)  (msr_cov) (0)  (0)  (0)
                                                      (0)   (0)   (0) (msr_cov) (0)  (0)
                                                      (0)   (0)   (0)  (0) (msr_cov) (0)
                                                      (0)   (0)   (0)  (0)  (0)  (msr_cov) ;

    return msg;
}





















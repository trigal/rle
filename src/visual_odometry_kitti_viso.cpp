#include <boost/assign.hpp>
#include "Utils.h"
#include <string>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <Eigen/Eigen>
#include <fstream>
#include <cstdlib>
#include <boost/filesystem.hpp>
#include <boost/array.hpp>
#include <dynamic_reconfigure/server.h>
#include <road_layout_estimation/visual_odometry_nvmConfig.h>

using namespace boost::filesystem;
using namespace std;



// vars -----------------------------------------------------------
tf::TransformBroadcaster* tfb_;
tf::TransformListener* tf_;
tf::Transform t(tf::createIdentityQuaternion(),tf::Vector3(0,0,0));

// ros publishers
ros::Publisher pose_publisher;
ros::Publisher odom_publisher;

/***************************************************************************************************************/

tf::Transform move_x(tf::createIdentityQuaternion(), tf::Vector3(0.5,0,0));
tf::Transform roll_transform(tf::createQuaternionFromRPY(-M_PI/2, 0, 0), tf::Vector3(0,0,0));
tf::Transform pitch_transform(tf::createQuaternionFromRPY(0, -M_PI/2, 0), tf::Vector3(0,0,0));
tf::Transform yaw_transform(tf::createQuaternionFromRPY(0, -M_PI/2, 0), tf::Vector3(0,0,0));


/**
 * @brief odometryCallback
 * @param msg
 */
void odometryCallback(const nav_msgs::Odometry& msg)
{
    ROS_INFO_STREAM("   Odometry msg arrived");

    // Generate PoseStamped message
    ros::Time current_time = ros::Time::now();
    geometry_msgs::PoseStamped pose;

    //      copy header
    pose.header.stamp = msg.header.stamp;
    pose.header.frame_id = msg.header.frame_id;
    pose.header.seq = msg.header.seq;

    //      get message tf from quaternion and position
    tf::Quaternion msg_quaternion; tf::quaternionMsgToTF(msg.pose.pose.orientation, msg_quaternion);
    tf::Transform t(msg_quaternion, tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z));

    //      pitch by -90°
    t = t * pitch_transform;

    //      update orientation
    pose.pose.orientation.x = t.getRotation().getX();
    pose.pose.orientation.y = t.getRotation().getY();
    pose.pose.orientation.z = t.getRotation().getZ();
    pose.pose.orientation.w = t.getRotation().getW();

    //      update position
    pose.pose.position.x = t.getOrigin().getX();
    pose.pose.position.y = t.getOrigin().getY();
    pose.pose.position.z = t.getOrigin().getZ();

    //      generate Odometry message
    nav_msgs::Odometry odometry;
    odometry = msg;
    odometry.header.frame_id = "robot_frame";
    odometry.child_frame_id = "odom_frame";
    double linear_err = 0; double angular_err = 0;
    double position_err = 0; double orientation_err = 0;
    odometry.twist.covariance = boost::assign::list_of  (linear_err) (0)   (0)  (0)  (0)  (0)
                                                        (0)  (linear_err)  (0)  (0)  (0)  (0)
                                                        (0)   (0)  (linear_err) (0)  (0)  (0)
                                                        (0)   (0)   (0) (angular_err) (0)  (0)
                                                        (0)   (0)   (0)  (0) (angular_err) (0)
                                                        (0)   (0)   (0)  (0)  (0)  (angular_err) ;

    odometry.pose.covariance = boost::assign::list_of  (position_err) (0)   (0)  (0)  (0)  (0)
                                                        (0)  (position_err)  (0)  (0)  (0)  (0)
                                                        (0)   (0)  (position_err) (0)  (0)  (0)
                                                        (0)   (0)   (0) (orientation_err) (0)  (0)
                                                        (0)   (0)   (0)  (0) (orientation_err) (0)
                                                        (0)   (0)   (0)  (0)  (0)  (orientation_err) ;
//    odometry.header.stamp = msg.header.stamp;
//    odometry.header.seq = msg.header.seq;
//    odometry.pose.pose = pose.pose;
//    odometry.pose.covariance = msg.pose.covariance;
//    odometry.twist.twist = msg.twist.twist;
//    odometry.twist.covariance = odometry.twist.covariance;

    //      publish PoseStamped
    pose_publisher.publish(pose);

    //      publish Odometry
    odom_publisher.publish(odometry);

    //      send TF
    tfb_->sendTransform(tf::StampedTransform(t, current_time, "robot_frame", "odom_frame"));

    //      print published msg
    //Utils::printPoseMsgToCout(pose);
}

/** ***********************************************************************************************************
 * @brief main
 *************************************************************************************************************/
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "visual_odometry_kitti_viso");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("VISUAL ODOMETRY KITTI-LibViso2 STARTED");

    // tf variables
    tfb_ = new tf::TransformBroadcaster();
    tf_ = new tf::TransformListener();

    // subscriber
    ros::Subscriber sub = nh.subscribe("/stereo_odometer/odometry", 1, odometryCallback);

    // publishers
    pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/visual_odometry/pose", 1);
    odom_publisher = nh.advertise<nav_msgs::Odometry>("/visual_odometry/odometry", 1);

    // spin
    ros::spin();

    return 0;
}


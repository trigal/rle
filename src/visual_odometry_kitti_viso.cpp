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
#include <road_layout_estimation/visual_odometry_kitti_visoConfig.h>

using namespace boost::filesystem;
using namespace std;

// vars -----------------------------------------------------------
tf::TransformBroadcaster* tfb_;
tf::TransformListener* tf_;
tf::StampedTransform t;

// ros publishers
ros::Publisher pose_publisher;
ros::Publisher odom_publisher;

// dynamic reconfigure parameters
bool enable_custom_uncertainty = false;
double position_uncertainty = 0;
double orientation_uncertainty = 0;
double linear_uncertainty = 0;
double angular_uncertainty = 0;
bool enable_custom_twist = false;

// store position
geometry_msgs::PoseStamped old_pose;

/***************************************************************************************************************/

/**
 * @brief buildOdomMsgFrom2Poses
 * @param old_pose
 * @param pose
 * @param pos_err
 * @param or_err
 * @param lin_err
 * @param ang_err
 * @return
 */
nav_msgs::Odometry buildOdomMsgFrom2Poses(const geometry_msgs::PoseStamped& old_pose,const geometry_msgs::PoseStamped& pose, double pos_err, double or_err, double lin_err, double ang_err)
{
    // calculate cov from uncertainty
    pos_err = pos_err*pos_err;
    or_err = or_err*or_err;
    lin_err = lin_err*lin_err;
    ang_err = ang_err*ang_err;

    // create odom msg
    nav_msgs::Odometry odom;
    odom.child_frame_id = "odom_frame";
    odom.header.frame_id = "robot_frame";
    odom.header.stamp = pose.header.stamp;

    odom.pose.pose = pose.pose;
    odom.twist.twist = Utils::getSpeedFrom2PoseStamped(old_pose, pose);

    odom.twist.covariance = boost::assign::list_of  (lin_err) (0)   (0)  (0)  (0)  (0)
                                                        (0)  (lin_err)  (0)  (0)  (0)  (0)
                                                        (0)   (0)  (lin_err) (0)  (0)  (0)
                                                        (0)   (0)   (0) (ang_err) (0)  (0)
                                                        (0)   (0)   (0)  (0) (ang_err) (0)
                                                        (0)   (0)   (0)  (0)  (0)  (ang_err) ;

    odom.pose.covariance = boost::assign::list_of  (pos_err) (0)   (0)  (0)  (0)  (0)
                                                        (0)  (pos_err)  (0)  (0)  (0)  (0)
                                                        (0)   (0)  (pos_err) (0)  (0)  (0)
                                                        (0)   (0)   (0) (or_err) (0)  (0)
                                                        (0)   (0)   (0)  (0) (or_err) (0)
                                                        (0)   (0)   (0)  (0)  (0)  (or_err) ;
    return odom;
}

/**
 * @brief reconfigureCallback
 * @param config
 * @param level
 */
void reconfigureCallback(road_layout_estimation::visual_odometry_kitti_visoConfig &config, uint32_t level) {
    // set measure uncertainty
    enable_custom_uncertainty = config.enable_custom_uncertainty;
    position_uncertainty = config.position_uncertainty;
    orientation_uncertainty = config.orientation_uncertainty;
    linear_uncertainty = config.linear_uncertainty;
    angular_uncertainty = config.angular_uncertainty;

    // speed calculation
    enable_custom_twist = config.enable_custom_twist;
}



/**
 * @brief odometryCallback
 * @param msg
 */
void odometryCallback(const nav_msgs::Odometry& msg)
{
    ROS_INFO_STREAM("   Odometry msg arrived");

    // wait for transform
    static tf::TransformListener listener;
    try{
        ros::Time now = ros::Time::now();
        ROS_INFO_STREAM("   before waiting");
//        ros::spinOnce();
//        listener.waitForTransform("/robot_frame", "/car", now, ros::Duration(1.0));
        ROS_INFO_STREAM("   before lookup");
        listener.lookupTransform("/robot_frame", "/car", ros::Time(0), t);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        return;
    }

    // send tf
    ROS_INFO_STREAM("   before sending");
    tfb_->sendTransform(tf::StampedTransform(t, t.stamp_, "robot_frame", "odom_frame"));
    ROS_INFO_STREAM("   after sending");

    // Generate PoseStamped message
    ros::Time current_time = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id ="robot_frame";
    pose.header.stamp = current_time;

    //      update orientation
    pose.pose.orientation.x = t.getRotation().getX();
    pose.pose.orientation.y = t.getRotation().getY();
    pose.pose.orientation.z = t.getRotation().getZ();
    pose.pose.orientation.w = t.getRotation().getW();

    //      update position
    pose.pose.position.x = t.getOrigin().getX();
    pose.pose.position.y = t.getOrigin().getY();
    pose.pose.position.z = t.getOrigin().getZ();

    nav_msgs::Odometry odometry;
    odometry.twist.twist = msg.twist.twist;
    odometry.twist.covariance = msg.twist.covariance;
    odometry.pose.covariance = msg.pose.covariance;
    odometry.pose.pose = pose.pose;
    odometry.header.stamp = current_time;
    odometry.header.frame_id = "robot_frame";
    odometry.child_frame_id = "odom_frame";

    // change covariance depending on selected flag
    if(enable_custom_uncertainty){
        double linear_err = linear_uncertainty * linear_uncertainty;
        double angular_err = angular_uncertainty * angular_uncertainty;
        double position_err = position_uncertainty * position_uncertainty;
        double orientation_err = orientation_uncertainty * orientation_uncertainty;
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
    }
    else
    {
        odometry.pose.covariance = msg.pose.covariance;
        odometry.twist.covariance = odometry.twist.covariance;
    }

    // store pose
    old_pose = pose;

    // publish PoseStamped
    pose_publisher.publish(pose);
    ROS_INFO_STREAM("   Pose published");

    // calculate speed with our formulas if flag is enabled
    if(enable_custom_twist){
        odometry = buildOdomMsgFrom2Poses(old_pose, pose, position_uncertainty, orientation_uncertainty, linear_uncertainty, angular_uncertainty);
        //  re-assign message covariance if 'custom uncertainty' is disabled
        if(!enable_custom_uncertainty){
            odometry.twist.covariance = msg.twist.covariance;
            odometry.pose.covariance = msg.pose.covariance;
        }
    }

    //      publish Odometry
    odom_publisher.publish(odometry);
    ROS_INFO_STREAM("   Odometry published");


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

    // init dynamic reconfigure
    dynamic_reconfigure::Server<road_layout_estimation::visual_odometry_kitti_visoConfig> server;
    dynamic_reconfigure::Server<road_layout_estimation::visual_odometry_kitti_visoConfig>::CallbackType f;
    f = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(f);

    // tf broadcaster
    tfb_ = new tf::TransformBroadcaster();

    // subscriber
    ros::Subscriber sub = nh.subscribe("/stereo_odometer/odometry", 1, odometryCallback);

    // publishers
    pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/visual_odometry/pose", 1);
    odom_publisher = nh.advertise<nav_msgs::Odometry>("/visual_odometry/odometry", 1);

    // spin
    ros::spin();

    return 0;
}


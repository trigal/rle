/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Author:    Dario Limongi                                              *
 *   Email:     dario.limongi@gmail.com                                    *
 *   Date:      22/01/2014                                                 *
 *                                                                         *
 ***************************************************************************/

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
tf::TransformListener *listener;

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
bool enable_lookup_twist = false;

// store position
geometry_msgs::PoseStamped old_pose;
bool first_run = true;
geometry_msgs::Twist frame_speed; // stores last frame speed
tf::StampedTransform frame_pose;  // stores last frame pose
State6DOF old_state;              // stores last state 6Dof (calculated from frame_speed and frame_pose)

/***************************************************************************************************************/

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
    enable_lookup_twist = config.enable_lookup_twist;
}



/**
 * @brief odometryCallback
 * @param msg
 */
void createState6DOF()
{
    // Set pose from origin
    old_state.setPose(Vector3d(frame_pose.getOrigin().getX(),frame_pose.getOrigin().getY(),frame_pose.getOrigin().getZ()));

    // Set rotation from pose rotation
    old_state.setRotation(AngleAxisd(
                              Quaterniond(
                                  frame_pose.getRotation().getW(),
                                  frame_pose.getRotation().getX(),
                                  frame_pose.getRotation().getY(),
                                  frame_pose.getRotation().getZ()
                                  )
                              )
                          );
    // Create vector3 from twist.linear
    old_state.setTranslationalVelocity(Vector3d(frame_speed.linear.x, frame_speed.linear.y, frame_speed.linear.z));

    // Get quaternion from twist.angular RPY
    tf::Matrix3x3 tmp_rot;
    tmp_rot.setEulerYPR(frame_speed.angular.z, frame_speed.angular.y, frame_speed.angular.x);
    tf::Quaternion q;
    tmp_rot.getRotation(q);
    old_state.setRotationalVelocity(AngleAxisd(Quaterniond(q.getW(),q.getX(), q.getY(), q.getZ())));
}

void odometryCallback(const nav_msgs::Odometry& msg)
{
    ROS_INFO_STREAM("   Visual Odometry: Libviso2 odometry msg arrived");
    try{
        listener->waitForTransform("/robot_frame", "/car", ros::Time(0), ros::Duration(0.1));
        listener->lookupTransform("/robot_frame", "/car", ros::Time(0), t);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        return;
    }

    // send tf
    tfb_->sendTransform(tf::StampedTransform(t, t.stamp_, "robot_frame", "odom_frame"));

    // Generate PoseStamped message
    ros::Time current_time = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id ="robot_frame";
    pose.header.stamp = current_time;
    pose.header.seq = msg.header.seq;

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

    // set old_pose same as pose if it's the first msg, this means that speed will be 0 this time (if custom_twist is enabled)
    if(first_run){
        old_pose = pose;
        first_run = false;
    }

    // publish PoseStamped
    pose_publisher.publish(pose);

    // calculate speed with our formulas if flag is enabled
    if(enable_custom_twist)
    {
        if(enable_lookup_twist)
        {
            try
            {
//                listener->waitForTransform("/odom_frame", "/robot_frame", ros::Time(0), ros::Duration(0.1));

                // Alternativa a lookupTransform
//                tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)),msg.header.stamp,"odom_frame");
//                tf::Stamped<tf::Pose> odom_pose;
//                listener->transformPose("robot_frame",ident,odom_pose);

                // Pose odom rispetto a robot_frame
                listener->lookupTransform("robot_frame", "odom_frame", msg.header.stamp, frame_pose);

                // Velocità odom_frame rispetto a robot_frame
                listener->lookupTwist("odom_frame", "robot_frame", "odom_frame", tf::Point(0,0,0), "odom_frame", ros::Time(0), ros::Duration(0.2), frame_speed);
                odometry.twist.twist = frame_speed;

                // Aggiorna il vecchio State6DOF data la nuova frame_pose e frame_speed
                createState6DOF();

            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                odometry.twist.twist = Utils::getSpeedFrom2PoseStamped(old_pose, pose);
            }
        }
        else{
            odometry.twist.twist = Utils::getSpeedFrom2PoseStamped(old_pose, pose);
        }
    }
    else {
        // Use libviso2 twist message
        odometry.twist.twist = msg.twist.twist;
    }

    // store pose
    old_pose = pose;

    //      publish Odometry
    odom_publisher.publish(odometry);
    ROS_INFO_STREAM("   Visual Odometry: odometry msg sent");

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

    listener = new tf::TransformListener();

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
    pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/visual_odometry/pose", 3);
    odom_publisher = nh.advertise<nav_msgs::Odometry>("/visual_odometry/odometry", 3);

    // spin
    ros::spin();

    return 0;
}


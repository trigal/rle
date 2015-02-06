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
tf::StampedTransform fixed_transform;
tf::StampedTransform old_transform, old_diocan;
tf::TransformListener *listener;


nav_msgs::Odometry old_msg;

// ros publishers
//ros::Publisher pose_publisher;
//ros::Publisher odom_publisher;
ros::Publisher delta_odom_publisher;

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


    tf::StampedTransform new_transform;
    new_transform.setOrigin(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z));
    new_transform.setRotation(tf::Quaternion( msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w));

    tfb_->sendTransform(tf::StampedTransform(new_transform, msg.header.stamp, "FANCULO", "CAMERE"));


    tf::StampedTransform t1,t2;
    tf::Transform t3;
    tf::Vector3 velculo;

    try
    {
        listener->lookupTransform("visual_odometry_odom_x_forward","odom",msg.header.stamp,fixed_transform);
        listener->lookupTwist("visual_odometry_car_frame", "visual_odometry_odom_x_forward", "visual_odometry_car_frame", tf::Point(0,0,0), "visual_odometry_car_frame", ros::Time(0), ros::Duration(.5), frame_speed);

        listener->lookupTransform("visual_odometry_odom_x_forward","visual_odometry_car_frame",msg.header.stamp,t1);
        listener->lookupTransform("visual_odometry_odom_x_forward","visual_odometry_car_frame",msg.header.stamp - ros::Duration(0.5),t2);
        t3 = t2.inverseTimes(t1);
        velculo = t3.getOrigin() / 0.5;
        Eigen::AngleAxisd tmp_rot_vel(Eigen::Quaterniond(t3.getRotation().getW(),t3.getRotation().getX(),t3.getRotation().getY(),t3.getRotation().getZ()));
        tmp_rot_vel.angle() /= 0.5;
        Eigen::Quaterniond cane(tmp_rot_vel);
        tf::Quaternion culo;
        culo = t3.getRotation().slerp(tf::createIdentityQuaternion(),0.5);
        cout << endl;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
    }


    tf::StampedTransform diocan = tf::StampedTransform(new_transform,msg.header.stamp,"","");
    tfb_->sendTransform(tf::StampedTransform(diocan, msg.header.stamp, "FANCULO", "DIOCAN"));
    tfb_->sendTransform(tf::StampedTransform(old_diocan, msg.header.stamp, "FANCULO", "DIOCAN_OLD"));

    new_transform = tf::StampedTransform(fixed_transform * new_transform,msg.header.stamp,"","");
    tfb_->sendTransform(tf::StampedTransform(new_transform, msg.header.stamp, "FANCULO", "CAMERE_X_FWD"));
    tfb_->sendTransform(tf::StampedTransform(old_transform, msg.header.stamp, "FANCULO", "CAMERE_X_FWD_OLD"));



    if(first_run)
    {
        old_diocan = diocan;
        old_transform = new_transform;
        first_run = false;
        return;
    }


    tf::Transform delta_transform;
    tf::Transform delta_transform2;
    tf::Transform delta_transform3;

    delta_transform = old_transform.inverseTimes(new_transform);

    delta_transform2 = fixed_transform * old_diocan.inverse() * diocan ;

    delta_transform3.setOrigin(fixed_transform * old_diocan.inverse() * diocan.getOrigin());
    delta_transform3.setBasis(old_diocan.inverse().getBasis() * diocan.getBasis());

//    delta_transform2.setBasis(fixed_transform.getBasis() * old_diocan.inverse().getBasis());
//    delta_transform2.setOrigin(fixed_transform * old_diocan.inverse() * diocan.getOrigin());

    tfb_->sendTransform(tf::StampedTransform(delta_transform, msg.header.stamp, "CAMERE_X_FWD_OLD","COLCAZZO"));


    tfb_->sendTransform(tf::StampedTransform(delta_transform2, msg.header.stamp, "FANCULO","DIOCAN_COLCAZZO"));
    tfb_->sendTransform(tf::StampedTransform(delta_transform3, msg.header.stamp, "FANCULO","DIOCAN_COLCAZZO2"));

    old_transform = new_transform;
    old_diocan = diocan;
    old_msg = msg;

//    delta_odom_publisher.publish();
















//    geometry_msgs::PoseStamped pose_in, pose_out;
//    tf::StampedTransform t,t1,t2;
//    tf::Transform t3;
//    pose_in.header = msg.header;
//    pose_in.pose = msg.pose.pose;

//    tf::Vector3 gigi1,gigi2;
//    tf::Vector3 velculo;

//        listener->transformPose("visual_odometry_odom_x_forward",msg.header.stamp,pose_in,"odom",pose_out);
//        listener->lookupTransform("visual_odometry_odom_x_forward","visual_odometry_car_frame",msg.header.stamp,t);

//        listener->lookupTransform("visual_odometry_odom_x_forward","visual_odometry_car_frame",msg.header.stamp,t1);
//        listener->lookupTransform("visual_odometry_odom_x_forward","visual_odometry_car_frame",msg.header.stamp - ros::Duration(0.5),t2);

//        t3 = t2.inverseTimes(t1);
//        velculo = t3.getOrigin() / 0.5;

//        listener->lookupTwist("visual_odometry_car_frame", "visual_odometry_odom_x_forward", "visual_odometry_car_frame", tf::Point(0,0,0), "visual_odometry_car_frame", ros::Time(0), ros::Duration(.5), frame_speed);
//        frame_speed = t * msg.twist.twist;
//        tf::Vector3 ident(0,0,0);
//        gigi1 = t * ident;
//        gigi2 = t.inverse() * ident;


//    try{
//        listener->waitForTransform("visual_odometry_odom_x_forward", "visual_odometry_car_frame", ros::Time(0), ros::Duration(0.1));
//        listener->lookupTransform("visual_odometry_odom_x_forward", "visual_odometry_car_frame", ros::Time(0), t);
//    }
//    catch (tf::TransformException &ex) {
//        ROS_ERROR("%s",ex.what());
//        return;
//    }


//    // send tf
//    tfb_->sendTransform(tf::StampedTransform(t, t.stamp_, "robot_frame", "odom_frame"));


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
//    pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/visual_odometry/pose", 3);
//    odom_publisher = nh.advertise<nav_msgs::Odometry>("/visual_odometry/odometry", 3);
    delta_odom_publisher = nh.advertise<nav_msgs::Odometry>("/delta_visual_odometry_x_forward", 3);
    // spin
    ros::spin();

    return 0;
}


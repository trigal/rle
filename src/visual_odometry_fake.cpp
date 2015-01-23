/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Author:    Dario Limongi                                              *
 *   Email:     dario.limongi@gmail.com                                    *
 *   Date:      29/07/2014                                                 *
 *                                                                         *
 ***************************************************************************/

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
#include <dynamic_reconfigure/server.h>
#include <road_layout_estimation/visual_odometry_fakeConfig.h>

using std::vector;
using namespace Eigen;
using namespace std;
using namespace ros;

// common variables for steps building ***********************************************************
tf::TransformBroadcaster* tfb_;
tf::TransformListener* tf_;
// ***********************************************************************************************

// settings  *************************************************************************************
double position_uncertainty = 0.05;     /// position uncertainty
double orientation_uncertainty = 0.05;  /// orientation uncertainty
double linear_uncertainty = 0.05;       /// linear speed uncertainty
double angular_uncertainty = 0.05;      /// angular speed uncertainty
// ***********************************************************************************************

ros::Rate * rate = NULL;

/**
 * @brief reconfigureCallback
 * @param config
 * @param level
 */
void reconfigureCallback(road_layout_estimation::visual_odometry_fakeConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request");
    ROS_INFO("Publishing rate: %f",
            config.odom_rate
           );

    // set node running rate
    if(rate != NULL)
        delete rate;
    rate = new ros::Rate(config.odom_rate);

    // set measure uncertainty
    position_uncertainty = config.position_uncertainty;
    orientation_uncertainty = config.orientation_uncertainty;
    linear_uncertainty = config.linear_uncertainty;
    angular_uncertainty = config.angular_uncertainty;
}

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
    ros::Publisher pub1 = nh.advertise<nav_msgs::Odometry>("/visual_odometry/odometry_no_error",1);
    ros::Publisher pub2 = nh.advertise<nav_msgs::Odometry>("/visual_odometry/odometry",1);
    ros::Publisher pub3 = nh.advertise<geometry_msgs::PoseArray>("/visual_odometry/single_pose_array",1);
    ros::Publisher pub4 = nh.advertise<geometry_msgs::PoseArray>("/visual_odometry/single_pose_array_no_err",1);

    // init dynamic reconfigure
    dynamic_reconfigure::Server<road_layout_estimation::visual_odometry_fakeConfig> server;
    dynamic_reconfigure::Server<road_layout_estimation::visual_odometry_fakeConfig>::CallbackType f;
    f = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(f);

    // Set current time for msg header
    ros::Time current_time;
    ros::Time previous_time;
    current_time = ros::Time::now();
    previous_time = ros::Time::now();

    // tf variables
    tfb_ = new tf::TransformBroadcaster();
    tf_ = new tf::TransformListener();

    // rotation set-up
    tf::Transform temp_t; //used for getting particle speed
    tf::Transform t(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)); //WORLD
    tf::Pose a,b,c;
    a.setOrigin(tf::Vector3(0,0,0)); a.setRotation(tf::createQuaternionFromRPY(0.1,0.1,0));
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
        //t.setRotation(t.getRotation().normalized());
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
        msg.twist.twist = Utils::getSpeed(previous_time, current_time, temp_t, t);        
        previous_time = current_time;

        // publish message on topic "visual_odometry/odometry_no_error"
        pub1.publish(msg);

        // publish message on topic "visual_odometry/odometry"
        nav_msgs::Odometry noisy_msg = Utils::addNoiseAndCovToOdom(msg,
                                                                   (position_uncertainty*position_uncertainty),
                                                                   (orientation_uncertainty*orientation_uncertainty),
                                                                   (linear_uncertainty*linear_uncertainty),
                                                                   (angular_uncertainty*angular_uncertainty)
                                                                   );
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
        rate->sleep();
    }
    if(rate != NULL)
        delete rate;
}


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
double angle_rate = 4.5f*3.14f/180.0f;
double msr_pose_err = 0.05*0.05;     /// measure uncertainty ^ 2
double msr_speed_err = 0.05*0.05;     /// measure uncertainty ^ 2
double change_direction = 5;

ros::Time current_time;
ros::Time last_msg_time;
double time_diff;
tf::TransformBroadcaster* tfb_;
tf::TransformListener* tf_;
nav_msgs::Odometry msg;
double current_speed;
unsigned int msg_num;

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
    ros::Publisher pub_x = nh.advertise<nav_msgs::Odometry>("visual_odometry_test/axis_x",1);
    ros::Publisher pub_y = nh.advertise<nav_msgs::Odometry>("visual_odometry_test/axis_y",1);
    ros::Publisher pub_z = nh.advertise<nav_msgs::Odometry>("visual_odometry_test/axis_z",1);
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("visual_odometry_test/odometry",1);

    // Node publish rate
    ros::Rate rate(publish_rate);

    // move on X, Y or Z axis with movement_rate transform
    tf::Transform move_x(tf::createIdentityQuaternion(), tf::Vector3(movement_rate,0,0));
    tf::Transform move_y(tf::createIdentityQuaternion(), tf::Vector3(0,movement_rate,0));
    tf::Transform move_z(tf::createIdentityQuaternion(), tf::Vector3(0,0,movement_rate));
    tf::Transform move_neg_x(tf::createIdentityQuaternion(), tf::Vector3(-movement_rate,0,0));
    tf::Transform move_neg_y(tf::createIdentityQuaternion(), tf::Vector3(0,-movement_rate,0));
    tf::Transform move_neg_z(tf::createIdentityQuaternion(), tf::Vector3(0,0,-movement_rate));
    tf::Transform rotate_yaw(tf::createQuaternionFromYaw(angle_rate), tf::Vector3(0,0,0));
    tf::Transform rotate_neg_yaw(tf::createQuaternionFromYaw(-angle_rate), tf::Vector3(0,0,0));
    tf::Transform rotate_pitch(tf::createQuaternionFromRPY(0, angle_rate, 0), tf::Vector3(0,0,0));
    tf::Transform rotate_neg_pitch(tf::createQuaternionFromRPY(0, -angle_rate, 0), tf::Vector3(0,0,0));
    tf::Transform rotate_roll(tf::createQuaternionFromRPY(angle_rate, 0, 0), tf::Vector3(0,0,0));
    tf::Transform rotate_neg_roll(tf::createQuaternionFromRPY(-angle_rate, 0, 0), tf::Vector3(0,0,0));


    // build axis message
    nav_msgs::Odometry msg_x = getXMsg();
    nav_msgs::Odometry msg_y = getYMsg();
    nav_msgs::Odometry msg_z = getZMsg();

    // tf variables
    tfb_ = new tf::TransformBroadcaster();
    tf_ = new tf::TransformListener();
    tf::Transform t(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)); //WORLD

    // --------- Publish first msg ------------------------------
    msg_num = 0;
    current_speed = movement_rate / publish_rate;
    current_time = ros::Time::now();

    // send transform_msg
    tfb_->sendTransform(tf::StampedTransform(t, current_time, "robot_frame", "odom_frame"));

    // init & send odom_msg
    last_msg_time = current_time;
    msg = createOdomMsgFromTF(t);

    pub.publish(msg);
    rate.sleep();
    // ----------------------------------------------------------

    while(ros::ok()){

        // apply transform
        if(msg_num >= 0 && msg_num<change_direction){
            t = t * move_x;
        }
        else if(msg_num>=change_direction && msg_num<(2*change_direction)){
            t = t * move_neg_x;
        }
        else if(msg_num >= (2*change_direction) && msg_num<(3*change_direction)){
            t = t * move_y;
        }
        else if(msg_num>=(3*change_direction) && msg_num<(4*change_direction)){
            t = t * move_neg_y;
        }
        else if(msg_num >= (4*change_direction) && msg_num<(5*change_direction)){
            t = t * move_z;
        }
        else if(msg_num>= (5*change_direction) && msg_num<(6*change_direction)){
            t = t * move_neg_z;
        }
        else if(msg_num>= (6*change_direction) && msg_num<(7*change_direction)){
            t = t * rotate_yaw;
        }
        else if(msg_num>= (7*change_direction) && msg_num<(8*change_direction)){
            t = t * rotate_neg_yaw;
        }
        else if(msg_num >= (8*change_direction) && msg_num< (9*change_direction)){
            t = t * rotate_pitch;
        }
        else if(msg_num>=(9*change_direction) && msg_num< (10*change_direction)){
            t = t * rotate_neg_pitch;
        }
        else if(msg_num >= (10*change_direction) && msg_num< (11*change_direction)){
            t = t * rotate_roll;
        }
        else if(msg_num>=(11*change_direction) && msg_num< (12*change_direction)){
            t = t * rotate_neg_roll;
        }
        else
            t = t * move_x;


        // update current time
        current_time = ros::Time::now();
        time_diff = current_time.toSec() - last_msg_time.toSec();
        current_speed = movement_rate / time_diff;

        // create msg from transform
        msg = createOdomMsgFromTF(t);

        // publish it
        tfb_->sendTransform(tf::StampedTransform(t, current_time, "robot_frame", "odom_frame"));

        std::cout << "--------------------------------------------------------------------------------" << endl;
        std::cout << "[ Time diff ] " << time_diff << endl;
        std::cout << "[ Sent msg " << msg_num << "]:" << std::endl;
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
        last_msg_time = current_time;
        rate.sleep();
        msg_num++;
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

    // apply transform
    if(msg_num >= 0 && msg_num<(change_direction)){
        msg.twist.twist.linear.x = current_speed;
    }
    else if(msg_num>=(change_direction) && msg_num< (2*change_direction)){
        msg.twist.twist.linear.x = -current_speed;
    }
    else if(msg_num >= (2*change_direction) && msg_num< (3*change_direction)){
        msg.twist.twist.linear.y = current_speed;
    }
    else if(msg_num>= (3*change_direction) && msg_num< (4*change_direction)){
        msg.twist.twist.linear.y = -current_speed;
    }
    else if(msg_num >= (4*change_direction) && msg_num< (5*change_direction)){
        msg.twist.twist.linear.z = current_speed;
    }
    else if(msg_num>=(5*change_direction) && msg_num< (6*change_direction)){
        msg.twist.twist.linear.z = -current_speed;
    }
    else if(msg_num >= (6*change_direction) && msg_num< (7*change_direction)){
        msg.twist.twist.angular.z = current_speed;
    }
    else if(msg_num>=(7*change_direction) && msg_num< (8*change_direction)){
        msg.twist.twist.angular.z = -current_speed;
    }
    else if(msg_num >= (8*change_direction) && msg_num< (9*change_direction)){
        msg.twist.twist.angular.y = current_speed;
    }
    else if(msg_num>=(9*change_direction) && msg_num< (10*change_direction)){
        msg.twist.twist.angular.y = -current_speed;
    }
    else if(msg_num >= (10*change_direction) && msg_num< (11*change_direction)){
        msg.twist.twist.angular.x = current_speed;
    }
    else if(msg_num>=(11*change_direction) && msg_num< (12*change_direction)){
        msg.twist.twist.angular.x = -current_speed;
    }
    else{
        msg.twist.twist.linear.x = current_speed;
    }


    // set covs (they will be constant for all the execution)
    msg.twist.covariance =  boost::assign::list_of  (msr_speed_err) (0)   (0)  (0)  (0)  (0)
                                                       (0)  (msr_speed_err)  (0)  (0)  (0)  (0)
                                                       (0)   (0)  (msr_speed_err) (0)  (0)  (0)
                                                       (0)   (0)   (0) (msr_speed_err) (0)  (0)
                                                       (0)   (0)   (0)  (0) (msr_speed_err) (0)
                                                       (0)   (0)   (0)  (0)  (0)  (msr_speed_err) ;

    msg.pose.covariance =  boost::assign::list_of  (msr_pose_err) (0)  (0)  (0)  (0)  (0)
                                                      (0) (msr_pose_err)   (0)  (0)  (0)  (0)
                                                      (0)   (0)  (msr_pose_err) (0)  (0)  (0)
                                                      (0)   (0)   (0) (msr_pose_err) (0)  (0)
                                                      (0)   (0)   (0)  (0) (msr_pose_err) (0)
                                                      (0)   (0)   (0)  (0)  (0)  (msr_pose_err) ;

    return msg;
}





















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
#include <boost/filesystem.hpp>

using namespace boost::filesystem;
using namespace std;

#define CAMERAS_NUMBER 360

// vars -----------------------------------------------------------
vector<geometry_msgs::PoseStamped> pose_vec;
double FREQUENCY=25;
double msr_err = 0.05 * 0.05;

/**
 * @brief loadNVM
 * @param path
 * @return
 */
int loadNVM(const char* path) {

    ROS_INFO("loadNVM started");

    std::ifstream reconstruction(path);

    if(reconstruction.is_open()) {
        ROS_INFO(".nvm file opened");
        ROS_INFO("Cameras found: %d", CAMERAS_NUMBER);

        std::string name[CAMERAS_NUMBER];
        double c[3];
        Eigen::Vector4d q, q1;
        Eigen::Matrix3d r;
        Eigen::Vector3d t;
        for(unsigned short int i = 0; i < CAMERAS_NUMBER; ++i) { //x ogni riga/camera

            reconstruction >> name[i];

            for(unsigned short int j = 0; j < 4; ++j) {
                reconstruction >> q[j];
            }
            for(unsigned short int j = 0; j < 3; ++j) {
                reconstruction >> c[j];
            }

            q1 = q;
            r = getRotationMatrix(q);
            t = getCameraCenterAfterRotation(c, r);

            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "robot_frame";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = t[0];
            pose.pose.position.y = t[1];
            pose.pose.position.z = t[2];
            pose.pose.orientation.w = q1[0];
            pose.pose.orientation.x = q1[1];
            pose.pose.orientation.y = q1[2];
            pose.pose.orientation.z = q1[3];

            pose_vec.push_back(pose);
        }
        return 0;

        reconstruction.close();
    }
    return false;
}

/**
 * @brief getSpeed
 * @param pose_prec
 * @param pose_t
 * @return
 */
geometry_msgs::Twist getSpeed(const geometry_msgs::PoseStamped & pose_prec, const geometry_msgs::PoseStamped & pose_t){
    geometry_msgs::Twist speed;
    double rate = 1 / FREQUENCY;

    // calculate linear speeds
    speed.linear.x = (pose_t.pose.position.x - pose_prec.pose.position.x) / rate;
    speed.linear.y = (pose_t.pose.position.y - pose_prec.pose.position.y) / rate;
    speed.linear.z = (pose_t.pose.position.z - pose_prec.pose.position.z) / rate;

    // Quaternion to RPY (step_prec)
    tf::Quaternion q1; tf::Quaternion q2;
    tf::quaternionMsgToTF(pose_prec.pose.orientation, q1);
    tf::Matrix3x3 m(q1);
    double roll_prec; double pitch_prec; double yaw_prec;
    m.getRPY(roll_prec, pitch_prec, yaw_prec);

    // Quaternion to RPY (step_t)
    tf::quaternionMsgToTF(pose_t.pose.orientation,q2);
    tf::Matrix3x3 m_t(q2);
    double roll_t; double pitch_t; double yaw_t;
    m_t.getRPY(roll_t, pitch_t, yaw_t);

    // calculate angular speeds
    speed.angular.x = ( angle_diff(roll_t, roll_prec) ) / rate;
    speed.angular.y = ( angle_diff(pitch_t, pitch_prec) ) / rate;
    speed.angular.z = ( angle_diff(yaw_t, yaw_prec) ) / rate;

    return speed;
}


/***************************************************************************************************************/
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "nvm_publisher");
    ros::NodeHandle nh;

    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("mapper/pose", 1);
    ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>("mapper/odometry", 1);
    ros::Publisher p_array_publisher = nh.advertise<geometry_msgs::PoseArray>("mapper/pose_array", 1);

    if(argc <= 1)
    {
        std::cout << "Give path of dataset file as parameter" << endl;
        return -1;
    }

    char * path;
    string argomento(argv[1]);
    path = realpath(argomento.c_str(),NULL);
    if(path){
        int error_type = loadNVM(path);

        if(error_type == 0) {
            ROS_INFO("Publishing Point Cloud and Pose");
        }

        ros::Rate loop_rate(FREQUENCY);

        geometry_msgs::PoseStamped old_pose = pose_vec[0];

        int i=0;
        while(ros::ok()) {

            // pose msg ---------------------------------------------------------------------------------------
            geometry_msgs::PoseStamped pose = pose_vec[i%CAMERAS_NUMBER];
            pose_publisher.publish(pose);

            // pose array msg ---------------------------------------------------------------------------------
            geometry_msgs::PoseArray p_array;
            p_array.poses.push_back(pose.pose);
            p_array.header.stamp = pose.header.stamp;
            p_array.header.frame_id = "robot_frame";
            p_array_publisher.publish(p_array);


            // odometry msg ----------------------------------------------------------------------------------
            nav_msgs::Odometry odom;
            odom.child_frame_id = "odom_frame";
            odom.header.frame_id = "robot_frame";
            odom.header.stamp = pose.header.stamp;
            odom.pose.pose = pose.pose;
            odom.twist.twist = getSpeed(old_pose, pose);

            odom.twist.covariance =  boost::assign::list_of  (msr_err) (0)   (0)  (0)  (0)  (0)
                                                                   (0)  (msr_err)  (0)  (0)  (0)  (0)
                                                                   (0)   (0)  (msr_err) (0)  (0)  (0)
                                                                   (0)   (0)   (0) (msr_err) (0)  (0)
                                                                   (0)   (0)   (0)  (0) (msr_err) (0)
                                                                   (0)   (0)   (0)  (0)  (0)  (msr_err) ;
            odom.pose.covariance =  boost::assign::list_of  (msr_err) (0)  ( 0)  (0)  (0)  (0)
                                                                  (0) (msr_err)   (0)  (0)  (0)  (0)
                                                                  (0)   (0)  (msr_err) (0)  (0)  (0)
                                                                  (0)   (0)   (0) (msr_err) (0)  (0)
                                                                  (0)   (0)   (0)  (0) (msr_err) (0)
                                                                  (0)   (0)   (0)  (0)  (0)  (msr_err) ;
            odom_publisher.publish(odom);
            // -----------------------------------------------------------------------------------------------

            std::cout << "--------------------------------------------------------------------------------" << endl;
            std::cout << "[ Sent msg " << i << "]:" << std::endl;
            std::cout << " Position:" << std::endl;
            std::cout << "  x: " << odom.pose.pose.position.x << std::endl;
            std::cout << "  y: " << odom.pose.pose.position.y << std::endl;
            std::cout << "  z: " << odom.pose.pose.position.z << std::endl;
            std::cout << " Orientation quaternion: " << std::endl;
            std::cout << "  w: " << odom.pose.pose.orientation.w << std::endl;
            std::cout << "  x: " << odom.pose.pose.orientation.x << std::endl;
            std::cout << "  y: " << odom.pose.pose.orientation.y << std::endl;
            std::cout << "  z: " << odom.pose.pose.orientation.z << std::endl;
            std::cout << " Linear speed: " << std::endl;
            std::cout << "  x: " << odom.twist.twist.linear.x << std::endl;
            std::cout << "  y: " << odom.twist.twist.linear.y << std::endl;
            std::cout << "  z: " << odom.twist.twist.linear.z << std::endl;
            std::cout << " Angular speed: " << std::endl;
            std::cout << "  x: " << odom.twist.twist.angular.x << std::endl;
            std::cout << "  y: " << odom.twist.twist.angular.y << std::endl;
            std::cout << "  z: " << odom.twist.twist.angular.z << std::endl;
            std::cout << std::endl;



            // aggiorno i valori
            old_pose = pose;
            i=i+1;
            loop_rate.sleep();
        }
    }
    else
    {
        cout << "DATASET FILE NOT FOUND, NODE WILL NOT RUN" << endl;
        return -1;
    }

    return 0;
}

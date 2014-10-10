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

using namespace boost::filesystem;
using namespace std;



// vars -----------------------------------------------------------
vector<geometry_msgs::PoseStamped> pose_vec;
tf::TransformBroadcaster* tfb_;
tf::TransformListener* tf_;
unsigned int CAMERAS_NUMBER;
double FREQUENCY=10;
double speed_err = 0.05 * 0.05;
double pose_err = 0.05 * 0.05;
double nvm_scale_factor = 0.09;
bool scale_factor_enabled = false;
/***************************************************************************************************************/
/**
 * @brief loadNVM
 * @param path
 * @return
 */
int loadNVM(const char* path) {
    std::ifstream reconstruction(path);

    if(reconstruction.is_open()) {
        ROS_INFO("Started reading file");

        double c[3];
        Eigen::Vector4d q, q1;
        Eigen::Matrix3d r;
        Eigen::Vector3d t;
        std::string name[10];
        unsigned int counter = 0;

        while(!reconstruction.eof())
        {
            counter++;

            //used for skipping fist character of every line
            reconstruction >> name[0];

            for(unsigned short int j = 0; j < 4; ++j) {
                reconstruction >> q[j];
            }
            for(unsigned short int j = 0; j < 3; ++j) {
                reconstruction >> c[j];
            }

            q1 = q;
            Eigen::Matrix3d m;
            m << 0,0,1,1,0,0,0,1,0;
            r = m*Utils::getRotationMatrix(q);
            t = Utils::getCameraCenterAfterRotation(c, r);

            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "robot_frame";
            pose.header.stamp = ros::Time::now();

            if(scale_factor_enabled)
            {
                pose.pose.position.x = t[0]*nvm_scale_factor;
                pose.pose.position.y = t[1]*nvm_scale_factor;
                pose.pose.position.z = t[2]*nvm_scale_factor;
            }
            else
            {
                pose.pose.position.x = t[0];
                pose.pose.position.y = t[1];
                pose.pose.position.z = t[2];
            }
            pose.pose.orientation.w = q1[0];
            pose.pose.orientation.x = q1[1];
            pose.pose.orientation.y = q1[2];
            pose.pose.orientation.z = q1[3];

            pose_vec.push_back(pose);
        }

        reconstruction.close();
        CAMERAS_NUMBER = counter;
    }
    return false;
}


nav_msgs::Odometry buildOdomMsgFrom2Poses(const geometry_msgs::PoseStamped& old_pose,const geometry_msgs::PoseStamped& pose, double speed_err, double pose_err)
{
    nav_msgs::Odometry odom;
    odom.child_frame_id = "odom_frame";
    odom.header.frame_id = "robot_frame";
    odom.header.stamp = pose.header.stamp;

    odom.pose.pose = pose.pose;
    odom.twist.twist = Utils::getSpeedFrom2PoseStamped(old_pose, pose);

    odom.twist.covariance = boost::assign::list_of  (speed_err) (0)   (0)  (0)  (0)  (0)
                                                        (0)  (speed_err)  (0)  (0)  (0)  (0)
                                                        (0)   (0)  (speed_err) (0)  (0)  (0)
                                                        (0)   (0)   (0) (speed_err) (0)  (0)
                                                        (0)   (0)   (0)  (0) (speed_err) (0)
                                                        (0)   (0)   (0)  (0)  (0)  (speed_err);
    odom.pose.covariance = boost::assign::list_of  (pose_err) (0)   (0)  (0)  (0)  (0)
                                                        (0)  (pose_err)  (0)  (0)  (0)  (0)
                                                        (0)   (0)  (pose_err) (0)  (0)  (0)
                                                        (0)   (0)   (0) (pose_err) (0)  (0)
                                                        (0)   (0)   (0)  (0) (pose_err) (0)
                                                        (0)   (0)   (0)  (0)  (0)  (pose_err);
    return odom;
}

nav_msgs::Odometry buildOdomMsgFrom1Pose(const geometry_msgs::PoseStamped& old_pose, double speed_err, double pose_err)
{
    nav_msgs::Odometry odom;

    odom.child_frame_id = "odom_frame";
    odom.header.frame_id = "robot_frame";
    odom.header.stamp = old_pose.header.stamp;
    odom.pose.pose = old_pose.pose;
    odom.twist.covariance = boost::assign::list_of  (speed_err) (0)   (0)  (0)  (0)  (0)
                                                        (0)  (speed_err)  (0)  (0)  (0)  (0)
                                                        (0)   (0)  (speed_err) (0)  (0)  (0)
                                                        (0)   (0)   (0) (speed_err) (0)  (0)
                                                        (0)   (0)   (0)  (0) (speed_err) (0)
                                                        (0)   (0)   (0)  (0)  (0)  (speed_err) ;
    odom.pose.covariance = boost::assign::list_of  (pose_err) (0)   (0)  (0)  (0)  (0)
                                                        (0)  (pose_err)  (0)  (0)  (0)  (0)
                                                        (0)   (0)  (pose_err) (0)  (0)  (0)
                                                        (0)   (0)   (0) (pose_err) (0)  (0)
                                                        (0)   (0)   (0)  (0) (pose_err) (0)
                                                        (0)   (0)   (0)  (0)  (0)  (pose_err) ;
    return odom;
}

/** ***********************************************************************************************************
 * @brief main
 *************************************************************************************************************/

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "visual_odometry_nvm");
    ros::NodeHandle nh;

    // publishers
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/visual_odometry_nvm/pose", 1);
    ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>("/visual_odometry_nvm/odometry", 1);

    // tf variables
    tfb_ = new tf::TransformBroadcaster();
    tf_ = new tf::TransformListener();
    tf::Transform t(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)); //WORLD

    // load nvm dataset file
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

        // build first msg ------------------------------------------------------------------------------------
        int i=0;
        geometry_msgs::PoseStamped old_pose = pose_vec[0];
        old_pose.header.stamp = ros::Time::now();
        old_pose.header.frame_id = "robot_frame";
        pose_publisher.publish(old_pose);

        // odometry msg ----------------------------------------------------------------------------------
        nav_msgs::Odometry odom = buildOdomMsgFrom1Pose(old_pose, speed_err, pose_err);
        // publish message
        odom_publisher.publish(odom);

        // write message on console
        std::cout << "[ Sent msg " << i << "]:" << std::endl;
        Utils::printOdomMsgToCout(odom);

        // update values
        i=i+1;
        loop_rate.sleep();

        // send transform
        Utils::sendTfFromPoseStamped(old_pose, tfb_);

        // ----------------------------------------------------------------------------------------------------



        while(ros::ok()) {

            // pose msg ---------------------------------------------------------------------------------------
            geometry_msgs::PoseStamped pose = pose_vec[i%CAMERAS_NUMBER];
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "robot_frame";
            pose_publisher.publish(pose);

            // odometry msg ----------------------------------------------------------------------------------
            nav_msgs::Odometry odom = buildOdomMsgFrom2Poses(old_pose, pose, speed_err, pose_err);
            odom_publisher.publish(odom);

            // send transform
            Utils::sendTfFromPoseStamped(pose, tfb_);

            // -----------------------------------------------------------------------------------------------

            std::cout << "--------------------------------------------------------------------------------" << endl;
            std::cout << "[ Time diff ] " << odom.header.stamp.toSec() - old_pose.header.stamp.toSec() << endl;
            std::cout << "[ Sent msg " << i << "]:" << std::endl;
            Utils::printOdomMsgToCout(odom);

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

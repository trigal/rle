/*
 * layout_manager.cpp
 *
 *  Created on: Jul 29, 2014
 *      Author: dario
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <math.h>
#include <boost/assign.hpp>
#include "particle/LayoutComponent_Building.h"
#include "particle/LayoutComponent_RoadLane.h"
#include "particle/LayoutComponent_Crossing.h"
#include "particle/MotionModel.h"
#include "particle/Particle.h"
#include "LayoutManager.h"
#include "Odometry.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include <vector>
using namespace Eigen;
using namespace std;

// layout-manager settings
int num_particles = 1;
double mtn_uncertainty = 0.05;
double measure_uncertainty = 0.05;

// init variables
bool first_msg = true;
int step = 0;
MatrixXd mtn_err = MatrixXd::Identity(12,12) * (mtn_uncertainty*mtn_uncertainty); /// used for initialize mtn_model
MatrixXd odom_err = MatrixXd::Identity(12,12) * (measure_uncertainty*measure_uncertainty); /// used for initialize visual_odometry
Odometry visual_odometry;
VectorXd p_pose = VectorXd::Zero(12);
MatrixXd p_sigma = MatrixXd::Zero(12,12);
MotionModel mtn_model(mtn_err);
LayoutManager layout_manager;
ros::Publisher array_pub;
vector<Particle> particle_set;
nav_msgs::Odometry old_msg; /// used for delta_t calculation by header.stamp difference
LayoutComponent_Building p_comp0;
LayoutComponent_RoadLane p_comp1;
LayoutComponent_Crossing p_comp2;

/******************************************************************************************************/
/**
 * Returns a message of type nav_msgs::Odometry given
 * a VectorXd as parameter
 * @param pose
 * @return
 */
nav_msgs::Odometry getOdomFromPoseAndSigma(const VectorXd& pose, const MatrixXd& sigma);

/**
 * Return a 12x1 VectorXd representing the pose given
 * a message of type nav_msgs::Odometryfrom
 * @param msg
 * @return
 */
VectorXd getPoseVectorFromOdom(const nav_msgs::Odometry& msg);

/**
 * @brief getCovFromOdom
 * @param msg
 * @return
 */
MatrixXd getCovFromOdom(const nav_msgs::Odometry& msg);

/**
 * @brief getPoseFromVector
 * @param msg
 * @return
 */
geometry_msgs::Pose getPoseFromVector(const VectorXd& msg);

/**
 * @brief getNoise
 * @param err
 * @return
 */
double getNoise(double err);

/**
 * @brief addOffsetToVectorXd
 * @param pose
 * @param position_err
 * @param orientation_err
 * @return
 */
VectorXd addOffsetToVectorXd(const VectorXd& pose, double position_err, double orientation_err, double speed_err);

/******************************************************************************************************/

/**
 **************************************************************************************************
 * Callback called on nav_msg::Odometry arrival
 * @param msg
 **************************************************************************************************
 */
void odometryCallback(const nav_msgs::Odometry& msg)
{
    cout << "--------------------------------------------------------------------------------" << endl;
    cout << "[step: " << step << "]" << endl; step++;


/** stampo misura arrivata ************************************************************************** */
    std::cout << " ******* MSG ARRIVED. *******" << std::endl;
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
/** ************************************************************************************************* */

    // if it's our first incoming odometry msg, just use it as particle-set poses initializer
    // (filter won't be called)
    if(first_msg){
        // generate particle set
        for(int i=0; i<num_particles; i++)
        {
            Particle part(i, p_pose, p_sigma, mtn_model);
            VectorXd new_pose = getPoseVectorFromOdom(msg);

            // our first particle will have same position of the odom msg
            if(i!=0){

                // add some random noise
                srand(time(0));
                new_pose = addOffsetToVectorXd(new_pose, measure_uncertainty, measure_uncertainty, measure_uncertainty);

                // update cov
                MatrixXd new_cov = getCovFromOdom(msg);
                part.setParticleSigma(new_cov);
            }

            // push the particle into the p_set
            part.setParticleState(new_pose);

            // let's add a sample component in same position of the particle
            p_comp0.particle_id = part.getId();
            p_comp0.component_id = 0;
            p_comp0.component_state = new_pose;
            p_comp0.component_cov = p_sigma;
            part.addComponent(&p_comp0);

            p_comp1.particle_id = part.getId();
            p_comp1.component_id = 1;
            p_comp1.component_state = new_pose;
            p_comp1.component_cov = p_sigma;
            part.addComponent(&p_comp1);

            p_comp2.particle_id = part.getId();
            p_comp2.component_id = 2;
            p_comp2.component_state = new_pose;
            p_comp2.component_cov = p_sigma;
            part.addComponent(&p_comp2);

            // push created particle into particle-set
            particle_set.push_back(part);

            // set layout manager particle set
            layout_manager.setCurrentLayout(particle_set);
        }
        first_msg=false;

        // update old_msg

        cout << "ciao1" << endl;
        old_msg = msg;
        cout << "ciao2" << endl;

        // publish it!
        geometry_msgs::PoseArray array_msg;
        array_msg.header.stamp = msg.header.stamp;
        array_msg.header.frame_id = "robot_frame";
        array_msg.poses.push_back(msg.pose.pose);
        array_pub.publish(array_msg);

        return;
    }

    // retrieve measurement from odometry
    VectorXd msr_pose = getPoseVectorFromOdom(msg);
    MatrixXd msr_cov = getCovFromOdom(msg);

    // calculate delta_t
    double p_delta = msg.header.stamp.toSec() - old_msg.header.stamp.toSec();
    LayoutManager::delta_t = p_delta;
    layout_manager.setMeasureState(msr_pose);
    layout_manager.setMeasureCov(msr_cov);

    // DEBUG:
    //cout << "[msr] [pose: " << msr_pose(0) << ", " <<  msr_pose(1) <<  ", " << msr_pose(2) << "] [orientation: " << msr_pose(3) <<  ", " << msr_pose(4) <<  ", " << msr_pose(5) << "] "<< endl;
    //cout << "[msr orientation: " << msr_pose(3) <<  ", " << msr_pose(4) <<  ", " << msr_pose(5) << "] "<< endl;
    //cout << "[msr] [vel: " << msr_pose(6) << ", " <<  msr_pose(7) <<  ", " << msr_pose(8) << ", " << msr_pose(9) <<  ", " << msr_pose(10) <<  ", " << msr_pose(11) << "] "<< endl;

	// call particle_estimation
    layout_manager.layoutEstimation();

    // --------------------------------------------------------------------------------------
    // BUILD POSEARRAY MSG
    geometry_msgs::PoseArray array_msg;
    array_msg.header.stamp = msg.header.stamp; //ros::Time::now();
    array_msg.header.frame_id = "robot_frame";

    vector<Particle> particles = layout_manager.getCurrentLayout();

    // Insert all particles inside msg
    for(int i = 0; i<num_particles; i++)
    {
        Particle p = particles.at(i);
        VectorXd p_pose = p.getParticleState();
        array_msg.poses.push_back( getPoseFromVector(p_pose) );
    }

    // Publish it!
    array_pub.publish(array_msg);


    /** stampo misura filtrata ************************************************************************** */
        nav_msgs::Odometry odom;
        Particle p = particles.at(0);
        odom = getOdomFromPoseAndSigma(p.getParticleState(), p.getParticleSigma());
        std::cout << " ******* FILTRO *******" << std::endl;
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
    /** ************************************************************************************************* */
}

/**
 *************************************************************************************************
 * @brief main
 *************************************************************************************************
 */
int main(int argc, char **argv)
{
	// init ROS and NodeHandle
	ros::init(argc, argv, "layout_manager");
	ros::NodeHandle n;

    // init layout manager variables
    layout_manager.setVisualOdometry(visual_odometry);
    layout_manager.visual_odometry.setErrorCovariance(odom_err);

    // init header timestamp
    old_msg.header.stamp = ros::Time::now();

	// init subscriber
    //ros::Subscriber sub = n.subscribe("visual_odometry/odom_no_error", 1, odometryCallback);
    //ros::Subscriber sub = n.subscribe("visual_odometry/odom", 1, odometryCallback);
    //ros::Subscriber sub = n.subscribe("mapper/odometry", 1, odometryCallback);
    ros::Subscriber sub = n.subscribe("visual_odom_test/test", 1, odometryCallback);

    ROS_INFO_STREAM("LAYOUT MANAGER STARTED, LISTENING TO: " << sub.getTopic());

    // init publishers
    array_pub = n.advertise<geometry_msgs::PoseArray>("layout_manager/particle_pose_array",1);
    ros::spin();
	return 0;
}
/**
**************************************************************************************************
*/


// ***********************************************************************************************
// FUNCTIONS IMPLEMENTATIONS
// ***********************************************************************************************
VectorXd getPoseVectorFromOdom(const nav_msgs::Odometry& msg)
{
	VectorXd pose = VectorXd::Zero(12);

	//state
	pose(0) = msg.pose.pose.position.x;
	pose(1) = msg.pose.pose.position.y;
	pose(2) = msg.pose.pose.position.z;

	//orientation
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, q);
	tf::Matrix3x3 m(q);

	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	pose(3) = roll;
	pose(4) = pitch;
	pose(5) = yaw;

	//speed
	pose(6) = msg.twist.twist.linear.x;
	pose(7) = msg.twist.twist.linear.y;
	pose(8) = msg.twist.twist.linear.z;
	pose(9) = msg.twist.twist.angular.x;
	pose(10) = msg.twist.twist.angular.y;
	pose(11) = msg.twist.twist.angular.z;

	return pose;
}

MatrixXd getCovFromOdom(const nav_msgs::Odometry& msg){
	MatrixXd pose_cov = MatrixXd::Zero(6,6);
	MatrixXd twist_cov = MatrixXd::Zero(6,6);
	MatrixXd cov = MatrixXd::Zero(12,12);

	// build cov array from boost::matrix
	double pose_cov_array [36];
	for(int i=0; i<36; i++){
		pose_cov_array[i] = msg.pose.covariance.elems[i];
	}

	// build cov array from boost::matrix
	double twist_cov_array [36];
	for(int i=0; i<36; i++){
		twist_cov_array[i] = msg.twist.covariance.elems[i];
	}

	// build the two 6x6 cov matrix from arrays
	int counter = 0;
	for(int row=0; row<6; row++)
	{
		for(int column=0; column<6; column++)
		{
			pose_cov(row,column) = pose_cov_array[counter];
			twist_cov(row,column) = twist_cov_array[counter];
			counter++;
		}
	}

	// join the two matrix on top-left and bottom-right corner of the 12x12 cov matrix
	cov.topLeftCorner(6,6) = pose_cov;
	cov.bottomRightCorner(6,6) = twist_cov;

	return cov;
}

nav_msgs::Odometry getOdomFromPoseAndSigma(const VectorXd& pose, const MatrixXd& sigma){

	// istantiate odom
	nav_msgs::Odometry odom;

	//position
	odom.pose.pose.position.x = pose(0);
	odom.pose.pose.position.y = pose(1);
	odom.pose.pose.position.z = pose(2);

	//orientation
	tf::Quaternion odom_quat = tf::createQuaternionFromRPY(double(pose(3)), double(pose(4)), double(pose(5)));
	odom.pose.pose.orientation.x = odom_quat.getX();
	odom.pose.pose.orientation.y = odom_quat.getY();
	odom.pose.pose.orientation.z = odom_quat.getZ();
	odom.pose.pose.orientation.w = odom_quat.getW();

	//speed
	odom.twist.twist.linear.x = pose(6);
	odom.twist.twist.linear.y = pose(7);
	odom.twist.twist.linear.z = pose(8);
	odom.twist.twist.angular.x = pose(9);
	odom.twist.twist.angular.y = pose(10);
	odom.twist.twist.angular.z = pose(11);

	//covariance
	MatrixXd pose_cov = MatrixXd::Zero(6,6);
	MatrixXd twist_cov = MatrixXd::Zero(6,6);
	pose_cov = sigma.topLeftCorner(6,6);
	twist_cov = sigma.bottomRightCorner(6,6);

	odom.pose.covariance = boost::assign::list_of
			(double(pose_cov(0,0)))   (double(pose_cov(0,1)))  (double(pose_cov(0,2)))   (double(pose_cov(0,3)))   (double(pose_cov(0,4)))  (double(pose_cov(0,5)))
			(double(pose_cov(1,0)))   (double(pose_cov(1,1)))  (double(pose_cov(1,2)))   (double(pose_cov(1,3)))   (double(pose_cov(1,4)))  (double(pose_cov(1,5)))
			(double(pose_cov(2,0)))   (double(pose_cov(2,1)))  (double(pose_cov(2,2)))   (double(pose_cov(2,3)))   (double(pose_cov(2,4)))  (double(pose_cov(2,5)))
			(double(pose_cov(3,0)))   (double(pose_cov(3,1)))  (double(pose_cov(3,2)))   (double(pose_cov(3,3)))   (double(pose_cov(3,4)))  (double(pose_cov(3,5)))
			(double(pose_cov(4,0)))   (double(pose_cov(4,1)))  (double(pose_cov(4,2)))   (double(pose_cov(4,3)))   (double(pose_cov(4,4)))  (double(pose_cov(4,5)))
			(double(pose_cov(5,0)))   (double(pose_cov(5,1)))  (double(pose_cov(5,2)))   (double(pose_cov(5,3)))   (double(pose_cov(5,4)))  (double(pose_cov(5,5)));

	odom.twist.covariance = boost::assign::list_of
			(double(twist_cov(0,0)))   (double(twist_cov(0,1)))  (double(twist_cov(0,2)))   (double(twist_cov(0,3)))   (double(twist_cov(0,4)))  (double(twist_cov(0,5)))
			(double(twist_cov(1,0)))   (double(twist_cov(1,1)))  (double(twist_cov(1,2)))   (double(twist_cov(1,3)))   (double(twist_cov(1,4)))  (double(twist_cov(1,5)))
			(double(twist_cov(2,0)))   (double(twist_cov(2,1)))  (double(twist_cov(2,2)))   (double(twist_cov(2,3)))   (double(twist_cov(2,4)))  (double(twist_cov(2,5)))
			(double(twist_cov(3,0)))   (double(twist_cov(3,1)))  (double(twist_cov(3,2)))   (double(twist_cov(3,3)))   (double(twist_cov(3,4)))  (double(twist_cov(3,5)))
			(double(twist_cov(4,0)))   (double(twist_cov(4,1)))  (double(twist_cov(4,2)))   (double(twist_cov(4,3)))   (double(twist_cov(4,4)))  (double(twist_cov(4,5)))
			(double(twist_cov(5,0)))   (double(twist_cov(5,1)))  (double(twist_cov(5,2)))   (double(twist_cov(5,3)))   (double(twist_cov(5,4)))  (double(twist_cov(5,5)));


	return odom;
}


/**
 * @param err
 * @return a random number between -err and err
 */
double getNoise(double err){
    double noise = (-err) + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(err - (-err))));
    return noise;
}

/**
 * Adds a random noise to a VectorXd 12x1 representing particle's pose
 * @param pose
 * @param err
 */
VectorXd addOffsetToVectorXd(const VectorXd& pose, double position_offset, double orientation_offset, double speed_offset)
{
    VectorXd vec = pose;
    vec(0) += getNoise(position_offset);
    vec(1) += getNoise(position_offset);
    vec(2) += getNoise(position_offset);

    vec(3) += getNoise(orientation_offset); vec(3) = atan2(sin(vec(3)),cos(vec(3)));
    vec(4) += getNoise(orientation_offset); vec(4) = atan2(sin(vec(4)),cos(vec(4)));
    vec(5) += getNoise(orientation_offset); vec(5) = atan2(sin(vec(5)),cos(vec(5)));

    vec(6) += getNoise(speed_offset);
    vec(7) += getNoise(speed_offset);
    vec(8) += getNoise(speed_offset);
    vec(9) += getNoise(speed_offset);
    vec(10) += getNoise(speed_offset);
    vec(11) += getNoise(speed_offset);

    return vec;
}

geometry_msgs::Pose getPoseFromVector(const VectorXd& msg){
   geometry_msgs::Pose pose;

   //set position
   pose.position.x = msg(0);
   pose.position.y = msg(1);
   pose.position.z = msg(2);

   //normalize orientation
   double roll = atan2(sin(msg(3)),cos(msg(3)));
   double pitch = atan2(sin(msg(4)),cos(msg(4)));
   double yaw = atan2(sin(msg(5)),cos(msg(5)));

   //set orientation
   tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
   tf::quaternionTFToMsg(q, pose.orientation);

   return pose;
}

/*
 * layout_manager.cpp
 *
 *  Created on: Jul 29, 2014
 *      Author: dario
 */

#include "Utils.h"
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
#include <string>

using namespace Eigen;
using namespace std;

// TOPIC EXAMPLES
//"visual_odometry_nvm/odometry"
//"visual_odometry/odometry_no_error"
//"visual_odometry/odometry"
//"visual_odometry_test/odometry"

// layout-manager settings ----------------------------------------------------------------------------
unsigned int num_particles = 1;
double mtn_uncertainty = 0.05;
double measure_uncertainty = 0.05;

// vars -----------------------------------------------------------------------------------------------
bool first_msg = true; /// flag used for init particle-set
int step = 0;   /// stores the current road_layout_manager step
MatrixXd mtn_err = MatrixXd::Identity(12,12) * (mtn_uncertainty*mtn_uncertainty); /// used for initialize mtn_model
MatrixXd odom_err = MatrixXd::Identity(12,12) * (measure_uncertainty*measure_uncertainty); /// used for initialize visual_odometry

// layout manager components
Odometry odometry;
VectorXd p_pose = VectorXd::Zero(12);
MatrixXd p_sigma = MatrixXd::Zero(12,12);
MotionModel mtn_model(mtn_err);
LayoutManager layout_manager;
vector<Particle> particle_set;

// components
LayoutComponent_Building p_comp0;
LayoutComponent_RoadLane p_comp1;
LayoutComponent_Crossing p_comp2;

// publishers
ros::Publisher array_pub;
ros::Publisher layout_odom_pub;
nav_msgs::Odometry old_msg; /// used for delta_t calculation by header.stamp difference


/** *************************************************************************************************
 * Callback called on nav_msg::Odometry arrival
 * @param msg
 ***************************************************************************************************/

void odometryCallback(const nav_msgs::Odometry& msg)
{
    cout << "--------------------------------------------------------------------------------" << endl;
    cout << "[step: " << step << "]" << endl; step++;

    // stampo misura arrivata
    std::cout << " ******* MSG ARRIVED. *******" << std::endl;
    Utils::printOdomMsgToCout(msg);

    // if it's our first incoming odometry msg, just use it as particle-set poses initializer
    // (filter won't be called)
    if(first_msg){
        // generate particle set
        for(int i=0; i<num_particles; i++)
        {
            Particle part(i, p_pose, p_sigma, mtn_model);
            VectorXd new_pose = Utils::getPoseVectorFromOdom(msg);

            // our first particle will have same position of the odom msg
            if(i!=0){
                // add some random noise
                new_pose = Utils::addOffsetToVectorXd(new_pose, measure_uncertainty, measure_uncertainty, measure_uncertainty);

                // update cov
                MatrixXd new_cov = Utils::getCovFromOdom(msg);
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
        old_msg = msg;

        // publish it!
        geometry_msgs::PoseArray array_msg;
        array_msg.header.stamp = msg.header.stamp;
        array_msg.header.frame_id = "robot_frame";
        array_msg.poses.push_back(msg.pose.pose);
        array_pub.publish(array_msg);

        return;
    }

    // retrieve measurement from odometry
    VectorXd msr_pose = Utils::getPoseVectorFromOdom(msg);
    MatrixXd msr_cov = Utils::getCovFromOdom(msg);

    // calculate delta_t
    LayoutManager::delta_t = msg.header.stamp.toSec() - old_msg.header.stamp.toSec();

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

        geometry_msgs::Pose pose = Utils::getPoseFromVector(p_pose);

        // normalize quaternion
        tf::Quaternion q;
        tf::quaternionMsgToTF(pose.orientation,q);
        q = q.normalize();
        tf::quaternionTFToMsg(q, pose.orientation);

        // push it!
        array_msg.poses.push_back( pose );
    }

    // Publish it!
    array_pub.publish(array_msg);

    // Update old_msg with current one for next step delta_t calculation
    old_msg = msg;

    // Print filtered out msr
    nav_msgs::Odometry odom;
    Particle p = particles.at(0);
    odom = Utils::getOdomFromPoseAndSigma(p.getParticleState(), p.getParticleSigma());
    odom.header.stamp = msg.header.stamp;
    odom.header.frame_id = "robot_frame";
    odom.child_frame_id = "odom_frame";
    std::cout << " ******* FILTRO *******" << std::endl;
    Utils::printOdomMsgToCout(odom);

    // publish odom
    layout_odom_pub.publish(odom);
}

/** ************************************************************************************************
 * @brief main
 **************************************************************************************************/
int main(int argc, char *argv[])
{
	// init ROS and NodeHandle
    ros::init(argc, argv, "road_layout_estimation");
	ros::NodeHandle n;

    // init subscriber
    if(argc <= 1)
    {
        ROS_INFO_STREAM("NO ODOMETRY TOPIC GIVEN AS ARGUMENT, NODE WILL NOT RUN");
        ROS_INFO_STREAM("Example:");
        ROS_INFO_STREAM("/visual_odometry/odometry");
        ROS_INFO_STREAM("/visual_odometry/odometry_no_error");
        ROS_INFO_STREAM("/visual_odometry_nvm/odometry");
        ROS_INFO_STREAM("/visual_odometry_test/odometry");
        return -1;
    }
    string argomento(argv[1]);
    ros::Subscriber sub = n.subscribe(argomento, 1, odometryCallback);
    ROS_INFO_STREAM("ROAD LAYOUT ESTIMATION STARTED, LISTENING TO: " << sub.getTopic());

    // init layout manager variables
    layout_manager.setOdometry(odometry);
    layout_manager.odometry.setErrorCovariance(odom_err);

    // init header timestamp
    old_msg.header.stamp = ros::Time::now();

    // init publishers
    array_pub = n.advertise<geometry_msgs::PoseArray>("layout_manager/particle_pose_array",1);
    layout_odom_pub = n.advertise<nav_msgs::Odometry>("layout_manager/odom",1);
    ros::spin();
	return 0;
}


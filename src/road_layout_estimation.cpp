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

// layout-manager settings ----------------------------------------------------------------------------
unsigned int num_particles = 1;
double mtn_uncertainty = 0.05;
double measure_uncertainty = 0.05;

// components
LayoutComponent_Building p_comp0;
LayoutComponent_RoadLane p_comp1;
LayoutComponent_Crossing p_comp2;

/** ************************************************************************************************
 * @brief main
 **************************************************************************************************/
int main(int argc, char *argv[])
{
	// init ROS and NodeHandle
    ros::init(argc, argv, "road_layout_estimation");
    ros::NodeHandle node_handle("~");

    // init subscriber
    if(argc <= 1)
    {
        ROS_INFO_STREAM("NO ODOMETRY TOPIC GIVEN AS ARGUMENT, THIS NODE WILL NOT RUN");
        ROS_INFO_STREAM("Examples:");
        ROS_INFO_STREAM("/visual_odometry/odometry");
        ROS_INFO_STREAM("/visual_odometry/odometry_no_error");
        ROS_INFO_STREAM("/visual_odometry_nvm/odometry");
        ROS_INFO_STREAM("/visual_odometry_test/odometry");
        return -1;
    }
    string topic(argv[1]);

    ROS_INFO_STREAM("ROAD LAYOUT ESTIMATION STARTED");

    // init layout_components
    vector<LayoutComponent*> layout_components;
    layout_components.push_back(&p_comp0);
    layout_components.push_back(&p_comp1);
    layout_components.push_back(&p_comp2);

    // init layout_manager
    LayoutManager layout_manager(node_handle, topic, num_particles, mtn_uncertainty, measure_uncertainty, layout_components);

    ros::spin();
	return 0;
}


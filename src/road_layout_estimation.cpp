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
#include <dynamic_reconfigure/server.h>
#include <road_layout_estimation/road_layout_estimationConfig.h>

using namespace Eigen;
using namespace std;

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

    // init layout_components
    vector<LayoutComponent*> layout_components;
    layout_components.push_back(&p_comp0);
    layout_components.push_back(&p_comp1);
    layout_components.push_back(&p_comp2);

    // init subscriber
    string argomento = "/visual_odometry_nvm/odometry";
    if(argc > 2)
    {
        ROS_INFO_STREAM("NO ODOMETRY TOPIC GIVEN AS ARGUMENT, NODE WILL NOT RUN");
        ROS_INFO_STREAM("Example:");
        ROS_INFO_STREAM("/visual_odometry/odometry");
        ROS_INFO_STREAM("/visual_odometry/odometry_no_error");
        ROS_INFO_STREAM("/visual_odometry_nvm/odometry");
        ROS_INFO_STREAM("/visual_odometry_test/odometry");
        return -1;
    }
    else{
        if(argc == 2){
            argomento = argv[1];
        }
    }

//    string argomento(argv[1]);

    // init layout_manager
    LayoutManager layout_manager(node_handle, argomento, layout_components);

    // Publish current layout
//    vector<Particle> particles = layout_manager.getCurrentLayout();
//    geometry_msgs::PoseArray array_msg = layout_manager.buildPoseArrayMsg(particles);
//    array_msg.header.stamp = ros::Time::now();
//    array_msg.header.frame_id = "map";
//    layout_manager.array_pub.publish(array_msg);

    ros::spin();

	return 0;
}


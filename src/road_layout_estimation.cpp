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
#include "MeasurementModel.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <road_layout_estimation/road_layout_estimationConfig.h>

using namespace Eigen;
using namespace std;

/** ************************************************************************************************
 * @brief main
 **************************************************************************************************/

int main(int argc, char *argv[])
{
	// init ROS and NodeHandle
    ros::init(argc, argv, "road_layout_estimation");
    ros::NodeHandle node_handle;

//    //-------------------------------------------------------------------------------------
//    cout << "----------------------------------------------------------------------------------" << endl;
//    tf::Quaternion q1 = tf::createQuaternionFromYaw(M_PI - 0.05);
//    tf::Quaternion q2 = tf::createQuaternionFromYaw(-M_PI + 0.05);
//    cout << "q1: " << q1.x() << " " << q1.y() << " " << q1.z() << " " << q1.w() << endl;
//    cout << "q2: " << q2.x() << " " << q2.y() << " " << q2.z() << " " << q2.w() << endl;

//    tf::Quaternion diff1 = q1.inverse() * q2;
//    tf::Quaternion diff2 = q2.inverse() * q1;

//    cout << "diff1: " << diff1.x() << " " << diff1.y() << " " << diff1.z() << " " << diff1.w() << endl;
//    cout << "diff2: " << diff2.x() << " " << diff2.y() << " " << diff2.z() << " " << diff2.w() << endl;
//    cout << "angle1: " << diff1.getAngle() << endl;
//    cout << "angle2: " << diff2.getAngle() << endl;
//    cout << "norm angle1: " << Utils::normalize_angle(diff1.getAngle()) << endl;
//    cout << "norm angle2: " << Utils::normalize_angle(diff2.getAngle()) << endl;

//    cout << "----------------------------------------------------------------------------------" << endl;
//    //-------------------------------------------------------------------------------------

    // init subscriber
    std::cout << "argc: " << argc << endl;
    string argomento = "/stereo_odometer/odometry";
    if(argc > 2)
    {
        ROS_INFO_STREAM("NO ODOMETRY TOPIC GIVEN AS ARGUMENT, NODE WILL NOT RUN");
        ROS_INFO_STREAM("Example:");
        ROS_INFO_STREAM("/stereo_odometer/odometry");
        ROS_INFO_STREAM("/visual_odometry/odometry");
        ROS_INFO_STREAM("/visual_odometry_nvm/odometry");
        return -1;
    }
    else{
        if(argc == 2){
            argomento = argv[1];
        }
    }

    // init layout_manager
    LayoutManager layout_manager(node_handle, argomento);

    ros::spin();

	return 0;
}


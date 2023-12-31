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
    ros::NodeHandle node_handle("/road_layout_estimation");
    //ros::AsyncSpinner spinner(1);

    std::cout << "Detected " << argc << " arguments (argc), printing values:" << endl;
    for (int i = 0; i < argc; i++)
        std::cout << "Argument " << i << ":" << argv[i] << endl;

    std::cout << endl;

    string visual_odometry_topic = "/stereo_odometer/odometry";
    string bagfile = "kitti_01";

    if (argc > 3)
    {
        ROS_INFO_STREAM("NO ODOMETRY TOPIC GIVEN AS ARGUMENT, NODE WILL NOT RUN");
        ROS_INFO_STREAM("Example:");
        return -1;
    }
    else
    {
        if (argc == 3)
        {
            visual_odometry_topic = argv[1];
            bagfile = argv[2];
            ROS_INFO_STREAM ("BAGFILE = " << bagfile);
        }
    }

    double rle_frequency = 1.0f;
    node_handle.param("rle_frequency"  , rle_frequency, 1.0);
    ROS_INFO_STREAM("RLE framework main loop frequency (Hz): " << rle_frequency);
    double timerInterval = 1.0f / rle_frequency; //0.05f;//0.05f; //(sec) 30Hz, 3xlibviso

    // Debug,
    // Info,
    // Warn,
    // Error,
    // Fatal,

    // init layout_manager
    LayoutManager layout_manager(node_handle, visual_odometry_topic, bagfile, timerInterval, ros::console::levels::Warn);

    ros::spin();
    //spinner.start();
    ros::waitForShutdown();

    return 0;
}


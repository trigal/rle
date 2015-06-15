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

#include <road_layout_estimation/getAllParticlesLatLon.h>
#include <ira_open_street_map/getHighwayInfo.h>
#include <ira_open_street_map/latlon_2_xy.h>
#include <ira_open_street_map/snap_particle_xy.h>

#include <road_layout_estimation/msg_roadState.h>

using std::vector;
using namespace Eigen;
using namespace std;
using namespace ros;

// common variables for steps building ***********************************************************
tf::TransformBroadcaster* tfb_;
tf::TransformListener* tf_;
// ***********************************************************************************************

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roadStateComponent_fake");
    ros::NodeHandle node_handler;

    double freq=0.75f;
    ros::Rate loop_rate(freq);
    ROS_INFO_STREAM("roadStateComponent_fake Started with freq: " << freq << " Hz");

    ros::ServiceClient callAllParticleLatLon;
    ros::ServiceClient callLatLonToXY;
    ros::ServiceClient callSnapParticleXy;
    callAllParticleLatLon = node_handler.serviceClient<road_layout_estimation::getAllParticlesLatLon>("/road_layout_estimation/layout_manager/getAllParticlesLatLon");
    callLatLonToXY        = node_handler.serviceClient<ira_open_street_map::latlon_2_xy>("/ira_open_street_map/latlon_2_xy");
    callSnapParticleXy    = node_handler.serviceClient<ira_open_street_map::snap_particle_xy>("/ira_open_street_map/snap_particle_xy");

    ros::ServiceClient callHighwayInfo;
    callHighwayInfo = node_handler.serviceClient<ira_open_street_map::getHighwayInfo>("/ira_open_street_map/getHighwayInfo");
    ira_open_street_map::getHighwayInfo service_call_highway;

    ros::Publisher roadState_pub = node_handler.advertise<road_layout_estimation::msg_roadState>("/fakeDetector/roadState", 1);

    // Avoid this node runs without the needed services already online
    callAllParticleLatLon.waitForExistence();
    callHighwayInfo.waitForExistence();
    callLatLonToXY.waitForExistence();
    callSnapParticleXy.waitForExistence();

    while(ros::ok())
    {
        ROS_INFO_STREAM("> START cycly, fake detector...");

        road_layout_estimation::msg_roadState roadState_message;
        road_layout_estimation::getAllParticlesLatLon service_call_getAllParticles;

        if(callAllParticleLatLon.call(service_call_getAllParticles))
        {
            ROS_INFO_STREAM("N# of particles: " << service_call_getAllParticles.response.particles);
            for (int i=0;i<service_call_getAllParticles.response.particles;i++)
            {
                ROS_INFO_STREAM("Particle Lat:\t" << service_call_getAllParticles.response.latitudes.at(i));
                ROS_INFO_STREAM("Particle Lon:\t" << service_call_getAllParticles.response.longitudes.at(i));

                ///
                /// SNAP PARTICLE LAT LON
                ///     1. convert lat lon to XY
                ///     2. snap_particle_xy

                /// *1*
                ira_open_street_map::latlon_2_xy convertLatLonToXY;
                convertLatLonToXY.request.latitude  = service_call_getAllParticles.response.latitudes.at(i);
                convertLatLonToXY.request.longitude = service_call_getAllParticles.response.longitudes.at(i);
                if (callLatLonToXY.call(convertLatLonToXY))
                {
                    /// *2*
                    ira_open_street_map::snap_particle_xy snapParticle;
                    snapParticle.request.x=convertLatLonToXY.response.x;
                    snapParticle.request.y=convertLatLonToXY.response.y;
                    snapParticle.request.max_distance_radius = 200;
                    if (callSnapParticleXy.call(snapParticle))
                    {
                        service_call_highway.request.way_id = snapParticle.response.way_id;
                        if(callHighwayInfo.call(service_call_highway))
                        {
                            ROS_INFO_STREAM("wayID       \t" << snapParticle.response.way_id);
                            ROS_INFO_STREAM("Oneway:     \t" << service_call_highway.response.oneway);
                            ROS_INFO_STREAM("N# Lanes:   \t" << service_call_highway.response.number_of_lanes);
                            ROS_INFO_STREAM("Road Width: \t" << service_call_highway.response.width);

                            roadState_message.oneway          = service_call_highway.response.oneway;
                            roadState_message.number_of_lanes = service_call_highway.response.number_of_lanes;
                            roadState_message.width           = service_call_highway.response.width;
                            roadState_message.way_id          = snapParticle.response.way_id;

                            roadState_pub.publish(roadState_message);
                        }
                        else
                            ROS_ERROR_STREAM("ERROR FAKE COMPONENT @ callHighwayInfo");
                    }
                    else
                        ROS_ERROR_STREAM("ERROR FAKE COMPONENT @ snapParticleXY");
                }
                else
                    ROS_ERROR("ERROR FAKE COMPONENT @ convertLatLon2XY");
            }

        }
        else
        {
            ROS_WARN("nope");
        }

        ros::spinOnce();

        ROS_INFO_STREAM("< END cycly, fake detector...");
        loop_rate.sleep();
    }
}


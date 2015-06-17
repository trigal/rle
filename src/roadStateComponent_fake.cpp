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
#include <boost/math/distributions/poisson.hpp>

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

    double maxScore = -1; //dummy method to send the fake-detection using the most weighted particle
    int    particleUsed=1;

    while(ros::ok())
    {
        ROS_INFO_STREAM("> START cycly, fake detector...");

        // reset dummy variables;
        maxScore = -1;
        particleUsed=1;

        road_layout_estimation::msg_roadState roadState_message;
        road_layout_estimation::getAllParticlesLatLon service_call_getAllParticles;

        if(callAllParticleLatLon.call(service_call_getAllParticles))
        {
            ROS_INFO_STREAM("N# of particles: " << service_call_getAllParticles.response.particleNumber);
            for (int i=0;i<service_call_getAllParticles.response.particleNumber;i++)
            {
                if (service_call_getAllParticles.response.particleScores.at(i)>maxScore)
                    maxScore=service_call_getAllParticles.response.particleScores.at(i);
                else
                    continue;

                particleUsed = i+1;

                //ROS_INFO_STREAM("Particle Lat:\t" << service_call_getAllParticles.response.latitudes.at(i));
                //ROS_INFO_STREAM("Particle Lon:\t" << service_call_getAllParticles.response.longitudes.at(i));

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
                            //ROS_INFO_STREAM("wayID       \t" << snapParticle.response.way_id);
                            //ROS_INFO_STREAM("Oneway:     \t" << service_call_highway.response.oneway);
                            //ROS_INFO_STREAM("N# Lanes:   \t" << service_call_highway.response.number_of_lanes);
                            //ROS_INFO_STREAM("Road Width: \t" << service_call_highway.response.width);

                            roadState_message.oneway          = service_call_highway.response.oneway;
                            roadState_message.number_of_lanes = service_call_highway.response.number_of_lanes + (int)(Utils::box_muller(0,0.5));
                            roadState_message.width           = service_call_highway.response.width           +       Utils::box_muller(0,0.3);
                            roadState_message.way_id          = snapParticle.response.way_id;

                            if (roadState_message.number_of_lanes<0)    roadState_message.number_of_lanes   = 0;
                            if (roadState_message.width<0)              roadState_message.width             = abs(roadState_message.width);
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

            // This should publish only the fake-measure from the most scored particle!
            ROS_INFO_STREAM("wayID       \t" << roadState_message.way_id);
            ROS_INFO_STREAM("Oneway:     \t" << roadState_message.oneway);
            ROS_INFO_STREAM("N# Lanes:   \t" << roadState_message.number_of_lanes);
            ROS_INFO_STREAM("Road Width: \t" << roadState_message.width);
            ROS_INFO_STREAM("Using " << particleUsed << "/" << service_call_getAllParticles.response.particleNumber << " particle");
            roadState_pub.publish(roadState_message);

        }
        else
        {
            ROS_WARN("Can't call Layoutmanager service callAllParticleLatLon ");
        }

        ros::spinOnce();

        ROS_INFO_STREAM("< END cycly, fake detector...\n");
        loop_rate.sleep();
    }
}


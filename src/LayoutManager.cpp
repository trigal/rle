/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Author:    Dario Limongi                                              *
 *   Email:     dario.limongi@gmail.com                                    *
 *   Date:      25/05/2014                                                 *
 *                                                                         *
 ***************************************************************************/

#include "MeasurementModel.h"
#include "Utils.h"
#include "LayoutManager.h"
#include "particle/Particle.h"
#include "particle/LayoutComponent_Building.h"
#include "particle/LayoutComponent.h"
#include "eigenmultivariatenormal.hpp"
#include <sensor_msgs/NavSatFix.h>
#include "osm_cartography/snap_particle_xy.h"
#include "osm_cartography/latlon_2_xy.h"
#include "osm_cartography/xy_2_latlon.h"
#include "osm_cartography/get_closest_way_distance_utm.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
using Eigen::ArrayXd;
using Eigen::MatrixXd;
using namespace std;

bool LayoutManager::openstreetmap_enabled = false; /// check this flag if we want to initialize particle-set with OSM and GPS
double LayoutManager::delta_t = 1;      /// Initialize static member value for C++ compilation
bool LayoutManager::first_run = true;   /// flag used for initiliazing particle-set with gps
bool LayoutManager::first_msg = true;   /// first odometry msg flag
int LayoutManager::step = 0;            /// filter step counter

/**
 * @brief buildPoseArrayMsg
 * @param particles
 * @return
 */
geometry_msgs::PoseArray LayoutManager::buildPoseArrayMsg(std::vector<Particle>& particles)
{
    // init array_msg
    geometry_msgs::PoseArray array_msg;
    array_msg.header.frame_id = "local_map";

    // Insert all particles inside msg
    for(int i = 0; i<particles.size(); i++)
    {
        // build Pose from Particle
        Particle p = particles.at(i);
        geometry_msgs::Pose pose = p.getParticleState().toGeometryMsgPose();

        // normalize quaternion
        tf::Quaternion q;
        tf::quaternionMsgToTF(pose.orientation,q);
        q = q.normalize();
        tf::quaternionTFToMsg(q, pose.orientation);

        // push it!
        array_msg.poses.push_back( pose );
    }

    return array_msg;
}

/**
 * Main LayoutManager constructor
 * @param n 'road_layout_manager' NodeHandle
 * @param l_components vector of layout components
 */
LayoutManager::LayoutManager(ros::NodeHandle& n, std::string& topic){
    // set this node_handle as the same of 'road_layout_manager'
    node_handle = n;

    // init odometry subscriber
    odometry_sub = node_handle.subscribe(topic, 3, &LayoutManager::odometryCallback, this);
    ROS_INFO_STREAM("ROAD LAYOUT ESTIMATION STARTED, LISTENING TO: " << odometry_sub.getTopic());

    // init road_lane_detection subscriber
    road_lane_sub = node_handle.subscribe("/road_lane_detection/lanes", 3, &LayoutManager::roadLaneCallback, this);

    // init values
    step = 0;
    num_particles = 0;
    LayoutManager::first_msg = true;

    // init motion model
//    mtn_model = new MotionModel();
    measurement_model = new MeasurementModel();

    // init header timestamp
    old_msg.header.stamp = ros::Time::now();

    // init publisher
    LayoutManager::array_pub = n.advertise<geometry_msgs::PoseArray>("/road_layout_estimation/layout_manager/particle_pose_array",1);
    LayoutManager::gps_pub = n.advertise<geometry_msgs::PoseStamped>("/road_layout_estimation/layout_manager/gps_fix",1);

    // init ROS service client
    latlon_2_xy_client = n.serviceClient<osm_cartography::latlon_2_xy>("/osm_cartography/latlon_2_xy");
    xy_2_latlon_client = n.serviceClient<osm_cartography::xy_2_latlon>("/osm_cartography/xy_2_latlon");
    snap_particle_xy_client = n.serviceClient<osm_cartography::snap_particle_xy>("/osm_cartography/snap_particle_xy");
    get_closest_way_distance_utm_client = n.serviceClient<osm_cartography::get_closest_way_distance_utm>("/osm_cartography/get_closest_way_distance_utm");

    // init dynamic reconfigure
    f = boost::bind(&LayoutManager::reconfigureCallback, this, _1, _2);
    server.setCallback(f);
}


/**
 * Callback handling dyamic reconfigure
 * @param config
 * @param level
 */
void LayoutManager::reconfigureCallback(road_layout_estimation::road_layout_estimationConfig &config, uint32_t level)
{
    cout << "   Reconfigure callback" << endl;

    // update uncertainty values -----------------------------------------------------------------------------------
    for(int i=0; i<current_layout.size(); ++i){
        Particle* particle_ptr = &current_layout.at(i);
        MotionModel* mtn_model_ptr = particle_ptr->getMotionModelPtr();
        mtn_model_ptr->setErrorCovariance(
                    config.mtn_model_position_uncertainty,
                    config.mtn_model_orientation_uncertainty,
                    config.mtn_model_linear_uncertainty,
                    config.mtn_model_angular_uncertainty
                    );

        mtn_model_ptr->setPropagationError(
                    config.propagate_translational_vel_error_x,
                    config.propagate_translational_vel_error_y,
                    config.propagate_translational_vel_error_z,
                    config.propagate_rotational_vel_error
                    );
    }


    // WARNING in teoria questi due non sono più necessari.
    mtn_model.setErrorCovariance(
                config.mtn_model_position_uncertainty,
                config.mtn_model_orientation_uncertainty,
                config.mtn_model_linear_uncertainty,
                config.mtn_model_angular_uncertainty
                );
    mtn_model.setPropagationError(
                config.propagate_translational_vel_error_x,
                config.propagate_translational_vel_error_y,
                config.propagate_translational_vel_error_z,
                config.propagate_rotational_vel_error
                );

    measurement_model->setMeasureCov(
                config.msr_model_position_uncertainty,
                config.msr_model_orientation_uncertainty,
                config.msr_model_linear_uncertainty,
                config.msr_model_angular_uncertainty
                );

    // update particle-set number (only if this isn't the first run) ------------------------------------------------
    if(!LayoutManager::first_run){
        if(config.particles_number > num_particles)
        {
            // let's add some empty particles to particle-set:
           int counter = current_layout.size(); //this will keep track of current ID
           int particles_to_add = config.particles_number - num_particles;
           for(int i=0; i<particles_to_add; i++)
           {
               Particle part(counter, mtn_model);

               State6DOF tmp;
               tmp.addNoise(0.5, 0.5, 0.5, 0.5);

               part.setParticleState(tmp);

               // update particle score using OpenStreetMap
               if(LayoutManager::openstreetmap_enabled){
                   // Get particle state
                   Vector3d p_state = part.getParticleState().getPose();
                   geometry_msgs::PoseStamped pose_local_map_frame;
                   pose_local_map_frame.header.frame_id = "local_map";
                   pose_local_map_frame.header.stamp = ros::Time::now();
                   pose_local_map_frame.pose.position.x = p_state(0);
                   pose_local_map_frame.pose.position.y =  p_state(1);
                   pose_local_map_frame.pose.position.z =  p_state(2);

                   tf::Stamped<tf::Pose> tf_pose_map_frame, tf_pose_local_map_frame;
                   tf::poseStampedMsgToTF(pose_local_map_frame, tf_pose_local_map_frame);

                   // Transform pose from "local_map" to "map"
                   tf_listener.transformPose("map", ros::Time(0), tf_pose_local_map_frame, "local_map", tf_pose_map_frame);

                   // Build request
                   osm_cartography::get_closest_way_distance_utm srv;
                   srv.request.x = tf_pose_map_frame.getOrigin().getX();
                   srv.request.y = tf_pose_map_frame.getOrigin().getY();
                   srv.request.max_distance_radius = 200; // distance radius for finding the closest nodes for particle snap

                   // Get distance from closest street and set it as particle score
                   if (LayoutManager::get_closest_way_distance_utm_client.call(srv))
                   {
                       part.setParticleScore(srv.response.distance);
                   }
                   else
                   {
                       // Either service is down or particle is too far from a street
                       part.setParticleScore(9999999);
                   }
               }

               // Push particle
               current_layout.push_back(part);

               // Update counter
               counter = counter+1;
           }
        }
        else if(config.particles_number < num_particles)
        {
            // let's erase particles starting from particle-set tail
           int particles_to_remove = num_particles - config.particles_number;
           for(int i=0; i<particles_to_remove; i++){
               // delete last element
               current_layout.erase(current_layout.end());
           }
        }

        num_particles = config.particles_number;
    }

    // ---------------------------------------------------------------------------------------------------------
    if(LayoutManager::first_run){

        if(LayoutManager::openstreetmap_enabled)
        {
            ROS_INFO_STREAM("Road layout manager first run, init particle-set from GPS signal");

            // -------- WAIT FOR GPS MESSAGE ----------------------------------------- //

//            // wait for GPS message (coming from Android device)
//            sensor_msgs::NavSatFix::ConstPtr gps_msg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/fix");

//            // Save values into local vars (this is a trick when GPS device isn't available or we want to simulate a GPS msg)
//            double alt = gps_msg->altitude;
//            double lat = gps_msg->latitude;
//            double lon = gps_msg->longitude;
//            double cov1 = gps_msg->position_covariance[0];
//            double cov2 = gps_msg->position_covariance[4];

            // ------ simulated GPS msg ----------- //
            // via Chiese
//            double alt = 164.78;
//            double lat = 45.520172;
//            double lon = 9.217983;
//            double cov1 = 15;
//            double cov2 = 15;

//            // nodo mappa oneway
//            double alt = 164.78;
//            double lat = 45.5232719;
//            double lon = 9.2148104;
//            double cov1 = 15;
//            double cov2 = 15;

//            // U14
//            double alt = 164.78;
//            double lat = 45.5238;
//            double lon = 9.21954;
//            double cov1 = 15;
//            double cov2 = 15;

            // KITTI 01
            double alt = 164.78;
            double lat = 49.006719195871;
            double lon = 8.4893558806503;
            double cov1 = 15;
            double cov2 = 15;
            ros::Duration(0.5).sleep(); // sleep for 2 secs
                                        // (simulate gps time fix, this will give time to publish poses to Rviz, not needed for RViz 2d pose estimate)

            // Publish GPS fix on map
//            fix.header.frame_id = "map";
//            fix.header.stamp = ros::Time::now();
//            fix.pose.position.x = latlon_2_xy_srv.response.x;
//            fix.pose.position.y = latlon_2_xy_srv.response.y;
//            fix.pose.orientation.w = 1;
//            LayoutManager::gps_pub.publish(fix);
            // ------------------------ END OF GPS MSG ---------------------------------- //


            // ------------------------ WAIT FOR RVIZ INITIAL POSE  --------------------- //
//            cout << "   Click on '2D pose estimation' in RViz for initialize particle set" << endl;
//            geometry_msgs::PoseWithCovarianceStamped::ConstPtr rviz_msg = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/initialpose");

//            // Get PoseStamped from Rviz msg
//            geometry_msgs::PoseStamped pose_local_map_frame;
//            pose_local_map_frame.pose = rviz_msg->pose.pose;
//            pose_local_map_frame.header = rviz_msg->header;
//            pose_local_map_frame.header.stamp = ros::Time::now(); // Rviz msg has no stamp

//            // Create tf::Pose
//            tf::Stamped<tf::Pose> tf_pose_map_frame, tf_pose_local_map_frame;
//            tf::poseStampedMsgToTF(pose_local_map_frame, tf_pose_local_map_frame);

//            // Transform pose from "map" to "local_map"
//            tf_listener.transformPose("map", ros::Time(0), tf_pose_local_map_frame, "local_map", tf_pose_map_frame);

//            // convert UTM to LatLon
//            osm_cartography::xy_2_latlon xy_2_latlon_srv;
//            xy_2_latlon_srv.request.x = tf_pose_map_frame.getOrigin().getX();
//            xy_2_latlon_srv.request.y = tf_pose_map_frame.getOrigin().getY();
//            if (!LayoutManager::xy_2_latlon_client.call(xy_2_latlon_srv)){
//                ROS_ERROR("   Failed to call 'xy_2_latlon' service");
//                return;
//            }

//            double alt = 0;
//            double lat = xy_2_latlon_srv.response.latitude;
//            double lon = xy_2_latlon_srv.response.longitude;
//            double cov1 = 0;
//            double cov2 = 0;
            // ------------------------ END RVIZ INITIAL POSE  ------------------------- //

            // Get XY values from GPS coords
            osm_cartography::latlon_2_xy latlon_2_xy_srv;
            latlon_2_xy_srv.request.latitude = lat;
            latlon_2_xy_srv.request.longitude = lon;
            geometry_msgs::Point point;
//            geometry_msgs::PoseStamped fix; // used for GPS publisher
            if (LayoutManager::latlon_2_xy_client.call(latlon_2_xy_srv))
            {
                 point.x = latlon_2_xy_srv.response.x;
                 point.y = latlon_2_xy_srv.response.y;
            }
            else
            {
              ROS_ERROR("   Failed to call 'latlon_2_xy_srv' service");
              return;
            }

            // Set mean
            Eigen::Vector2d mean;
            mean << point.x, point.y;

            // Set covariance
            Eigen::Matrix2d covar = Eigen::Matrix2d::Identity();
            covar(0,0) = cov1;
            covar(1,1) = cov2;

            cout << "------------------------------------------------------------" << endl;
            cout << "GPS FIX COORDINATES:" << endl;
            cout << "   lat: " << lat << " lon: " << lon << " alt: " << alt << endl;
            cout << "   x: " << boost::lexical_cast<std::string>(point.x) << " y: " << boost::lexical_cast<std::string>(point.y) << endl;
            cout << endl;
            cout << "MULTIVARIATE PARAMS: " << endl;
            cout << "   MEAN: " << endl;
            cout << "       " << boost::lexical_cast<std::string>(mean(0)) << endl;
            cout << "       " << boost::lexical_cast<std::string>(mean(1)) << endl;
            cout << "   COV: " << endl;
            cout << covar << endl;
            cout << "------------------------------------------------------------" << endl;

            // Create a bivariate gaussian distribution of doubles.
            // with our chosen mean and covariance
            Eigen::EigenMultivariateNormal<double, 2> normX(mean,covar);

            // Reset current_layout
            LayoutManager::current_layout.clear();

            // Populate current_layout with valid particles
            int particle_id = 1;
            int while_ctr = 1; // while loop infinite cycle prevenction
            while((while_ctr < 1000) && (current_layout.size() < config.particles_number))
            {
                if(while_ctr == 999){
                    cout << "   Random particle set init: while ctr reached max limit" << endl;
                    /**
                     * TODO: max_radius_size * 2 and find again
                     */
                }
                // Generate a sample from the bivariate Gaussian distribution
                Matrix<double,2,-1> sample = normX.samples(1);


                //-------------- SNAP PARTICLE v2  -------------------------------------- //
                // Init OSM cartography service
                osm_cartography::snap_particle_xy srv;
                srv.request.x = sample(0);
                srv.request.y = sample(1);
                srv.request.max_distance_radius = 200; // distance radius for finding the closest nodes for particle snap

                // Call snap particle service
                if (LayoutManager::snap_particle_xy_client.call(srv))
                {
                    geometry_msgs::PoseStamped pose_map_frame;
                    pose_map_frame.header.frame_id = "map";
                    pose_map_frame.header.stamp = ros::Time::now();
                    pose_map_frame.pose.position.x = srv.response.snapped_x;
                    pose_map_frame.pose.position.y = srv.response.snapped_y;
                    pose_map_frame.pose.position.z = 0.0;
                    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(srv.response.way_dir_rad),pose_map_frame.pose.orientation);

                    tf::Stamped<tf::Pose> tf_pose_map_frame, tf_pose_local_map_frame;
                    tf::poseStampedMsgToTF(pose_map_frame,tf_pose_map_frame);

                    // Transform pose from "map" to "local_map"
                    tf_listener.transformPose("local_map", ros::Time(0), tf_pose_map_frame, "map", tf_pose_local_map_frame);

                    // Init particle's pose
                    State6DOF p_pose;
                    p_pose._pose = Vector3d(tf_pose_local_map_frame.getOrigin().getX(), tf_pose_local_map_frame.getOrigin().getY(), 0);

                    // Create rotation object
                    AngleAxisd rotation = AngleAxisd(
                                Quaterniond(
                                    tf_pose_local_map_frame.getRotation().getW(),
                                    tf_pose_local_map_frame.getRotation().getX(),
                                    tf_pose_local_map_frame.getRotation().getY(),
                                    tf_pose_local_map_frame.getRotation().getZ()
                                    ));
                    p_pose.setRotation(rotation);

                    // Init particle's sigma
                    MatrixXd p_sigma = mtn_model.getErrorCovariance();

                    // Create particle and set its score
                    Particle part(particle_id, p_pose, p_sigma, mtn_model);
                    part.setParticleScore(srv.response.distance_from_way);

                    // Push particle into particle-set
                    current_layout.push_back(part);

                    // Update particles id counter
                    particle_id += 1;

                    // Check if we should create 2 particles with opposite direction
                    if(srv.response.way_dir_opposite_particles){

                        // Invert Yaw direction
                        p_pose.setRotation(Eigen::AngleAxisd(M_PI + part.getParticleState().getRotation().angle(),part.getParticleState().getRotation().axis()));

                        // Create particle and set its score
                        Particle opposite_part(particle_id, p_pose, p_sigma, mtn_model);
                        part.setParticleScore(srv.response.distance_from_way);

                        // Push particle inside particle-set
                        current_layout.push_back(opposite_part);

                        // Update particles id counter
                        particle_id += 1;
                    }
                }
                else
                {
                    ROS_ERROR("     Failed to call 'snap_particle_xy' service");
                    return;
                }

                // prevent infinite loop
                while_ctr += 1;
            } // end while loop for particles generations
        } // end if(gps_initialization)
        else
        {
            // gps initialization disabled, just generate particles with all values set to zero
            for(int i = 0; i < config.particles_number; i++){
                Particle zero_part(i, mtn_model);
                MatrixXd tmp_cov = mtn_model.getErrorCovariance();
                zero_part.setParticleSigma(tmp_cov);
                current_layout.push_back(zero_part);
            }
        }


        // Update particle_set size
        LayoutManager::num_particles = config.particles_number;

        // Update first_run flag
        LayoutManager::first_run = false;


        // print particle poses
        cout << "Random initialized particle set: " << endl;
        vector<Particle> particles = current_layout;
        for(int i = 0; i<particles.size(); i++)
        {
            // build Pose from Particle
            Particle p = particles.at(i);
            geometry_msgs::Pose pose = p.getParticleState().toGeometryMsgPose();

            // normalize quaternion
            tf::Quaternion q;
            tf::quaternionMsgToTF(pose.orientation,q);
            q = q.normalize();
            tf::quaternionTFToMsg(q, pose.orientation);

            // build posestamped
            geometry_msgs::PoseStamped pose_stamp;
            pose_stamp.header.stamp = ros::Time::now();
            pose_stamp.header.frame_id = "local_map";
            pose_stamp.pose = pose;

            // print it!
            cout << "Particle ID: " << p.getId() << endl;
            Utils::printPoseMsgToCout(pose_stamp);
            cout << endl;
        }// end print random particles


        // BUILD POSEARRAY MSG
        // Get particle-set
        geometry_msgs::PoseArray array_msg = LayoutManager::buildPoseArrayMsg(particles);
        array_msg.header.stamp = ros::Time::now();
        cout << "poses size: " << array_msg.poses.size() << endl;
        array_msg.header.frame_id = "local_map";
        cout << "frame id: " << array_msg.header.frame_id.c_str() << endl;
        // Publish it!
        LayoutManager::array_pub.publish(array_msg);

    } // end if(first_run)
}// end reconfigure callback


/** *************************************************************************************************
 * Callback called on nav_msg::Odometry arrival
 * @param msg
 ***************************************************************************************************/
void LayoutManager::odometryCallback(const nav_msgs::Odometry& msg)
{
    cout << "--------------------------------------------------------------------------------" << endl;
    cout << "[step: " << step << "]" << endl; step++;

    // stampo misura arrivata
    std::cout << " ******* MSG ARRIVED. *******" << std::endl;
    Utils::printOdomAngleAxisToCout(msg);

    // if it's our first incoming odometry msg, just use it as particle-set poses initializer
    // (filter won't be called)
    if(LayoutManager::first_msg && current_layout.size() == 0){
        cout << "   First Odometry message arrived, generating particles set" << endl;
        return;
    }

    // update flag
    LayoutManager::first_msg = false;

    // retrieve measurement from odometry
    State6DOF(measurement_model->getOldMsg()).printState("[old_msg]");
    measurement_model->setMsg(msg);

    // calculate delta_t
    delta_t = msg.header.stamp.toSec() - old_msg.header.stamp.toSec();

    // call particle_estimation
    vector<Particle>::iterator particle_itr;
    for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ ){

        // estimate particle
        (*particle_itr).particleEstimation(measurement_model);

        // update particle score using OpenStreetMap
        if(LayoutManager::openstreetmap_enabled){

            // Get particle state
            Vector3d p_state = (*particle_itr).getParticleState().getPose();
            geometry_msgs::PoseStamped pose_local_map_frame;
            pose_local_map_frame.header.frame_id = "local_map";
            pose_local_map_frame.header.stamp = ros::Time::now();
            pose_local_map_frame.pose.position.x = p_state(0);
            pose_local_map_frame.pose.position.y =  p_state(1);
            pose_local_map_frame.pose.position.z =  p_state(2);

            tf::Stamped<tf::Pose> tf_pose_map_frame, tf_pose_local_map_frame;
            tf::poseStampedMsgToTF(pose_local_map_frame, tf_pose_local_map_frame);

            // Transform pose from "local_map" to "map"
            tf_listener.transformPose("map", ros::Time(0), tf_pose_local_map_frame, "local_map", tf_pose_map_frame);

            // Build request
            osm_cartography::get_closest_way_distance_utm srv;
            srv.request.x = tf_pose_map_frame.getOrigin().getX();
            srv.request.y = tf_pose_map_frame.getOrigin().getY();
            srv.request.max_distance_radius = 200; // distance radius for finding the closest nodes for particle snap

            // Get distance from closest street and set it as particle score
            if (LayoutManager::get_closest_way_distance_utm_client.call(srv))
            {
                (*particle_itr).setParticleScore(srv.response.distance);
            }
            else
            {
                // Either service is down or particle is too far from a street
                (*particle_itr).setParticleScore(9999999);
            }

            cout << "   Particle score: " << (*particle_itr).getParticleScore() << endl;
        }
    }

    // --------------------------------------------------------------------------------------
    // BUILD POSEARRAY MSG
    // Get particle-set
    vector<Particle> particles = current_layout;
    geometry_msgs::PoseArray array_msg = LayoutManager::buildPoseArrayMsg(particles);
    array_msg.header.stamp = msg.header.stamp;

    // Publish it!
    array_pub.publish(array_msg);

    // Update old_msg with current one for next step delta_t calculation
    old_msg = msg;
}


/**
 * Check if car has moved or not by confronting odometry matrix and motion threshold matrix
 * @return true if car has moved beyond the the threshold matrix
 */
bool LayoutManager::checkHasMoved(){
	//MatrixXd diff_matrix = (visual_odometry.getOdometry().array() > motion_threshold).cast<double>();
//	MatrixXd diff_matrix = MatrixXd::Ones(12,12);
//	return diff_matrix.count() > 0; //every position over the threshold will count as 1

    return true;
}

/**
 * @brief LayoutManager::roadLaneCallback
 * @param msg
 */
void LayoutManager::roadLaneCallback(const road_lane_detection::road_lane_array& msg){
    // Add it to all particles
    for(int i=0; i<current_layout.size(); ++i){

        // Get all layout components of particle
        Particle* particle = &current_layout.at(i);
        vector<LayoutComponent*>* layout_components = particle->getLayoutComponentsPtr();

        // Clear old layout_components
        layout_components->clear();

        // Cycle through all lanes
        for(int k=0; k<msg.road_lane_vector.size(); ++k){

            // Create a new roadlane component
            road_lane_detection::road_lane lane = msg.road_lane_vector.at(k);

            LayoutComponent_RoadLane * road_lane = new LayoutComponent_RoadLane();
            road_lane->setParticleId(particle->getId());
            road_lane->setComponentId(layout_components->size());
            road_lane->setK1(lane.k1);
            road_lane->setK3(lane.k3);
            road_lane->setHomographyYResolution(msg.homography_y_resolution);
            road_lane->setTimestamp(msg.timestamp);

            // Add lane to layout components
            layout_components->push_back(road_lane);
        }
    }
}



/** **************************************************************************************************************/
/**
 * Estimate particles' components using particle filter
 */
void LayoutManager::componentsEstimation(){
    // STEP 1: SAMPLING (PREDICT COMPONENTS POSES)
    sampling();

    // STEP 2: PERTURBATE COMPONENT POSES
    componentsPerturbation();

    // STEP 3: WEIGHT LAYOUT-COMPONENTS
    calculateLayoutComponentsWeight();
}

/**
 * PARTICLE FILTER, STEP 1:
 * cicle through all the particles, and call their function "propagate-components"
 */
void LayoutManager::sampling(){
	vector<Particle>::iterator itr;
	for( itr = current_layout.begin(); itr != current_layout.end(); itr++ ){
		cout << "--- Propagating components of particle, ID: " << itr->getId() << " ---"<< endl;
        itr->propagateLayoutComponents();
	}
}

/**
 * PARTICLE FILTER, STEP 2:
 * @brief LayoutManager::componentsPerturbation
 */
void LayoutManager::componentsPerturbation(){

    // first, iterate over all particles of 'current_layout'
    for(int i=0; i<current_layout.size(); i++){
        Particle p = current_layout.at(i);
        vector<LayoutComponent*> vec = p.getLayoutComponents();

        // second, iterate over all layout-components of current 'particle'
        for(int j=0; j<vec.size(); j++){
            LayoutComponent* lc = vec.at(j);
            lc->componentPerturbation();
        }
    }
}

/**
 * PARTICLE FILTER, STEP 3:
 * @brief LayoutManager::calculateLayoutComponentsWeight
 *
 * andiamo a vedere quanto bene fitta la componente della singola particella nella realtà
 */
void LayoutManager::calculateLayoutComponentsWeight(){
    // first, iterate over all particles of 'current_layout'
    for(int i=0; i<current_layout.size(); i++){
        Particle p = current_layout.at(i);
        vector<LayoutComponent*> vec = p.getLayoutComponents();

        // second, iterate over all layout-components of current 'particle'
        for(int j=0; j<vec.size(); j++){
            LayoutComponent* lc = vec.at(j);
            lc->calculateWeight();
        }
    }
}
/** **************************************************************************************************************/

/** **************************************************************************************************************/
/**
 * FORMULA CALCOLO SCORE
 *
 * Cardinalità unaria (Particella presa INDIVIDUALMENTE)
 *  1- Kalman gain sulla pose della particella
 *  2- Somma dei WEIGHT delle varie componenti della particella (ottenuti dal filtraggio a particelle)
 *
 * Cardinalità >= 2
 *  1- plausibilità di esistenza tra le varie componenti di stesso tipo (due building sovrapposti ecc.) nella stessa particella
 *  2- plausibilità di esistenza tra componenti di diverso tipo (building sovrapposto a una macchina ecc.) nella stessa particella
 *
 * Nessuna particella verrà eliminata durante il procedimento di calcolo dello scoState6DOFre,
 * essa sarà mantenuta in vita nonostante lo score sia basso.
 * In questo modo si evita la possibilità di eliminare dal particle-set ipotesi plausibili che abbiano ricevuto
 * uno score di valore basso per motivi di natura diversa.
 *
 */
void LayoutManager::calculateScore(){
    vector<Particle>::iterator p_itr;
    for( p_itr = current_layout.begin(); p_itr != current_layout.end(); p_itr++ ){

        // calculate summatory of all components weights
        double components_weight = 0;
        vector<LayoutComponent*> vec = p_itr->getLayoutComponents();
        vector<LayoutComponent*>::iterator pc_itr;
        for(pc_itr = vec.begin(); pc_itr != vec.end(); pc_itr++){
            components_weight += (*pc_itr)->getComponentWeight();
        }

        // calculate unary score
        MatrixXd unary_score = p_itr->getKalmanGain();

        /// unary score con distanza dal segmento piu vicino
        //cout << "Particle ID: " << p_itr->getId() << ", unary score: "<< unary_score << endl;

        // calculate binary score
        MatrixXd binary_score = MatrixXd::Zero(12,12);

        // calculate particle score
        MatrixXd particle_score = unary_score * binary_score;
        //p_itr->setParticleScore(particle_score);
    }
}
/** **************************************************************************************************************/

vector<Particle> LayoutManager::layoutEstimation(){
	// Check if car has moved, if it has moved then estimate new layout
	if( checkHasMoved() )
	{
		// ----------------- predict and update layout poses using E.K.F ----------------- //
		vector<Particle>::iterator particle_itr;
		for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ ){
            (*particle_itr).particleEstimation(measurement_model);
		}
		// ------------------------------------------------------------------------------- //

		// -------------- sampling + perturbation + weight layout-components ------------- //
        //this->componentsEstimation();
		// ------------------------------------------------------------------------------- //

		// ------------------------------ calculate score -------------------------------- //
        //this->calculateScore();
		// ------------------------------------------------------------------------------- //

        // ------------------------------ resampling ------------------------------------- //
        if(new_detections)
		{
//			Add new candidate-components ----------------------------------------------------- //
//			(1) given N new candidate-layout-components duplicate 2^N particles and those particles to them
//			(2) calculate score
//			(3) resample all combination
		}
	}
	else
	{
		cout << endl <<  "Not moved!" << endl;
        //TODO: calculate score again, with new detections (if available)

	}


	return current_layout;
}


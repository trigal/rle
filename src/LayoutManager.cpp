/*
 * LayoutManager.cpp
 *
 *  Created on: May 25, 2014
 *      Author: dario
 */

#include "Odometry.h"
#include "Utils.h"
#include "LayoutManager.h"
#include "particle/Particle.h"
#include "particle/LayoutComponent_Building.h"
#include "particle/LayoutComponent.h"
#include "eigenmultivariatenormal.hpp"
#include "osm_cartography/is_valid_location_xy.h"
#include <fstream>
#include <vector>	//used for vector
#include <Eigen/Core>
#include <Eigen/Dense>	//used for motion threshold matrix
#include <iostream>
#include <math.h>
using Eigen::ArrayXd;
using Eigen::MatrixXd;
using namespace std;

double LayoutManager::delta_t = 1;      /// Initialize static member value for C++ compilation
bool LayoutManager::first_run = true;   /// flag used for initiliazing particle-set with gps

/**
 * Main LayoutManager constructor
 * @param n 'road_layout_manager' NodeHandle
 * @param l_components vector of layout components
 */
LayoutManager::LayoutManager(ros::NodeHandle& n, std::string& topic, vector<LayoutComponent*> l_components){
    // set this node_handle as the same of 'road_layout_manager'
    node_handle = n;

    // init subscriber
    sub = node_handle.subscribe(topic, 1, &LayoutManager::odometryCallback, this);
    ROS_INFO_STREAM("ROAD LAYOUT ESTIMATION STARTED, LISTENING TO: " << sub.getTopic());

    // init values
    step = 0;
    num_particles = 0;
    first_msg = true;
    layout_components = l_components;
    mtn_model.setErrorCovariance(0);
    odometry.setMeasureCov(0);

    // init header timestamp
    old_msg.header.stamp = ros::Time::now();

    // init publisher
    array_pub = n.advertise<geometry_msgs::PoseArray>("layout_manager/particle_pose_array",1);

    // init ROS service client
    service_client = n.serviceClient<osm_cartography::is_valid_location_xy>("is_valid_location_xy");

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
    // update errors values -----------------------------------------------------------------------------------------
    mtn_model.setErrorCovariance(
                config.mtn_model_position_uncertainty,
                config.mtn_model_orientation_uncertainty,
                config.mtn_model_linear_uncertainty,
                config.mtn_model_angular_uncertainty
                );
    odometry.setMeasureCov(
                config.msr_model_position_uncertainty,
                config.msr_model_orientation_uncertainty,
                config.msr_model_linear_uncertainty,
                config.msr_model_angular_uncertainty
                );

    // update particle-set number -----------------------------------------------------------------------------------
    if(!LayoutManager::first_run){
        if(config.particles_number > num_particles)
        {
            // let's add some empty particles to particle-set:
           int counter = current_layout.size(); //this will keep track of current ID
           int particles_to_add = config.particles_number - num_particles;
           for(int i=0; i<particles_to_add; i++)
           {
               VectorXd p_pose = VectorXd::Zero(12);
               MatrixXd p_sigma = MatrixXd::Zero(12,12);
               Particle part(counter, p_pose, p_sigma, mtn_model);
               current_layout.push_back(part);
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


    ROS_INFO("Particles Number: %d, Listening topic: %s",
           config.particles_number,
           sub.getTopic().c_str()
          );

    // ---------------------------------------------------------------------------------------------------------

    if(LayoutManager::first_run){
        ROS_INFO_STREAM("Road layout manager first run, init particle-set from GPS signal");
        // wait for GPS message (coming from Android device)
        // sensor_msgs::NavSatFix::ConstPtr gps_msg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/android/fix");

        // Simulate GPS msg
        sensor_msgs::NavSatFix gps_msg;
        boost::array<float,9> cov = {1000,0,0, 0,1000,0, 0,0,1000};
        gps_msg.position_covariance = cov;
        gps_msg.altitude = 264.78;
        gps_msg.latitude = 45.520172; //45.62183458;
        gps_msg.longitude = 9.217983; //9.19258087;

        // Get ECEF values from GPS coords
        geometry_msgs::Point point = Utils::lla2ecef(gps_msg.latitude, gps_msg.longitude, gps_msg.altitude);

        // Set mean
        Eigen::Vector2d mean;
        mean << point.x, point.y;

        // Set covariance
        Eigen::Matrix2d covar = Eigen::Matrix2d::Identity();
        covar(0,0) = gps_msg.position_covariance[0];
        covar(1,1) = gps_msg.position_covariance[4];

        cout << endl << "coordinates" << endl;
        cout << "lat: " << gps_msg.latitude << " lon: " << gps_msg.longitude << endl;
        cout << "mean: " << endl;
        cout << mean << endl << endl;
        cout << "cov: " << endl;
        cout << covar << endl << endl;

        // Create a bivariate gaussian distribution of doubles.
        // with our chosen mean and covariance
        Eigen::EigenMultivariateNormal<double, 2> normX(mean,covar);

        // Reset current_layout
        LayoutManager::current_layout.clear();

        // Populate current_layout with valid particles
        int particle_id = 1;
        while(current_layout.size() < config.particles_number)
        {
            // Generate a sample from the bivariate Gaussian distribution
            Matrix<double,2,-1> sample = normX.samples(1);
            cout << "x: " << sample(0) << " y: " << sample(1) << endl;

            // Init OSM cartography service
            osm_cartography::is_valid_location_xy srv;
            srv.request.x = sample(0);
            srv.request.y = sample(1);
            srv.request.max_distance_radius = 20;

            // Check if generated particle is next to a OSM map node
            if(LayoutManager::service_client.call(srv)){
                // Init particle's pose
                VectorXd p_pose = VectorXd::Zero(12);
                p_pose(0) = sample(0); // update X value
                p_pose(1) = sample(1); // update Y value

                // Init particle's sigma
                MatrixXd p_sigma = MatrixXd::Zero(12,12);

                // Push particle into particle-set
                Particle part(particle_id, p_pose, p_sigma, mtn_model);
                current_layout.push_back(part);

                // Update particles id counter
                particle_id += 1;
            }
        }

        // Update particle_set size
        LayoutManager::num_particles = config.particles_number;

        // Update first_run flag
        LayoutManager::first_run = false;

        // BUILD POSEARRAY MSG
        // Get particle-set
        vector<Particle> particles = current_layout;
        geometry_msgs::PoseArray array_msg = LayoutManager::buildPoseArrayMsg(particles);
        array_msg.header.stamp = ros::Time::now();

        // Publish it!
        array_pub.publish(array_msg);
    }
}



/**
 * @brief buildPoseArrayMsg
 * @param particles
 * @return
 */
geometry_msgs::PoseArray LayoutManager::buildPoseArrayMsg(std::vector<Particle>& particles)
{
    // init array_msg
    geometry_msgs::PoseArray array_msg;
    array_msg.header.frame_id = "robot_frame";

    // Insert all particles inside msg
    for(int i = 0; i<particles.size(); i++)
    {
        // build Pose from Particle
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

    return array_msg;
}
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
    Utils::printOdomMsgToCout(msg);

    // if it's our first incoming odometry msg, just use it as particle-set poses initializer
    // (filter won't be called)
    if(first_msg){
        //clear current layout
        current_layout.clear();

        // generate particle set
        for(int i=0; i<num_particles; i++)
        {
            // init an empty particle
            VectorXd p_pose = VectorXd::Zero(12);
            MatrixXd p_sigma = MatrixXd::Zero(12,12);
            Particle part(i, p_pose, p_sigma, mtn_model);

            // get pose & cov from odom msg
            VectorXd new_pose = Utils::getPoseVectorFromOdom(msg);
            MatrixXd new_cov = Utils::getCovFromOdom(msg);

            // add some random offset to particles (only the first one won't be noised)
            if(i!=1){
                new_pose = Utils::addOffsetToVectorXd(new_pose, 0.05, 0.05, 0.05);
            }

            // update particle values
            part.setParticleState(new_pose);
            part.setParticleSigma(new_cov);

            // let's add a sample component in same position of the particle
            layout_components.at(0)->particle_id = part.getId();
            layout_components.at(0)->component_id = 0;
            layout_components.at(0)->component_state = new_pose;
            layout_components.at(0)->component_cov = p_sigma;
            part.addComponent(layout_components.at(0));

            layout_components.at(1)->particle_id = part.getId();
            layout_components.at(1)->component_id = 1;
            layout_components.at(1)->component_state = new_pose;
            layout_components.at(1)->component_cov = p_sigma;
            part.addComponent(layout_components.at(1));

            layout_components.at(2)->particle_id = part.getId();
            layout_components.at(2)->component_id = 2;
            layout_components.at(2)->component_state = new_pose;
            layout_components.at(2)->component_cov = p_sigma;
            part.addComponent(layout_components.at(2));

            // push created particle into particle-set
            current_layout.push_back(part);
        }

        // update flag
        first_msg=false;

        // update old_msg
        old_msg = msg;

        // set odometry msg
        odometry.setMsg(msg);

        // publish it!
        geometry_msgs::PoseArray array_msg;
        array_msg.header.stamp = msg.header.stamp;
        array_msg.header.frame_id = "robot_frame";
        array_msg.poses.push_back(msg.pose.pose);
        array_pub.publish(array_msg);

        return;
    }

    // retrieve measurement from odometry
    odometry.setMsg(msg);

    // calculate delta_t
    delta_t = msg.header.stamp.toSec() - old_msg.header.stamp.toSec();

    // call particle_estimation
    layoutEstimation();

    // --------------------------------------------------------------------------------------
    // BUILD POSEARRAY MSG
    // Get particle-set
    vector<Particle> particles = current_layout;
    geometry_msgs::PoseArray array_msg = buildPoseArrayMsg(particles);
    array_msg.header.stamp = msg.header.stamp;

    // Publish it!
    array_pub.publish(array_msg);

    // Update old_msg with current one for next step delta_t calculation
    old_msg = msg;

    // Print filtered out measure
    nav_msgs::Odometry odom;
    Particle p = particles.at(0);
    odom = Utils::getOdomFromPoseAndSigma(p.getParticleState(), p.getParticleSigma());
    odom.header.stamp = msg.header.stamp;
    odom.header.frame_id = "robot_frame";
    odom.child_frame_id = "odom_frame";
    std::cout << " ******* FILTRO [particella ID: "<< p.getId() <<" (num. particelle: "<< current_layout.size() << ") *******" << std::endl;
    Utils::printOdomMsgToCout(odom);
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
 * Nessuna particella verrà eliminata durante il procedimento di calcolo dello score,
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
            (*particle_itr).particleEstimation(odometry);
		}
		// ------------------------------------------------------------------------------- //

		// -------------- sampling + perturbation + weight layout-components ------------- //
        //this->componentsEstimation();
		// ------------------------------------------------------------------------------- //

		// ------------------------------ calculate score -------------------------------- //
        //this->calculateScore();
		// ------------------------------------------------------------------------------- //


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


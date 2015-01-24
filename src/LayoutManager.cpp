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

#include "Odometry.h"
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
#include "osm_cartography/local_map_transform.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
    sub = node_handle.subscribe(topic, 3, &LayoutManager::odometryCallback, this);
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
    LayoutManager::array_pub = n.advertise<geometry_msgs::PoseArray>("/road_layout_estimation/layout_manager/particle_pose_array",1);
    LayoutManager::gps_pub = n.advertise<geometry_msgs::PoseStamped>("/road_layout_estimation/layout_manager/gps_fix",1);

    // init ROS service client
    latlon_2_xy_client = n.serviceClient<osm_cartography::latlon_2_xy>("/osm_cartography/latlon_2_xy");
    xy_2_latlon_client = n.serviceClient<osm_cartography::xy_2_latlon>("/osm_cartography/xy_2_latlon");
    local_map_tf_client = n.serviceClient<osm_cartography::local_map_transform>("/osm_latlon_converter/local_map_transform");
    snap_particle_xy_client = n.serviceClient<osm_cartography::snap_particle_xy>("/osm_cartography/snap_particle_xy");

    // init dynamic reconfigure
    f = boost::bind(&LayoutManager::reconfigureCallback, this, _1, _2);
    server.setCallback(f);

    // BUILD POSEARRAY MSG
    // Get particle-set
    vector<Particle> particles = current_layout;
    geometry_msgs::PoseArray array_msg = LayoutManager::buildPoseArrayMsg(particles);
    array_msg.header.stamp = ros::Time::now();
    array_msg.header.frame_id = "map";

    // Publish it!
    LayoutManager::array_pub.publish(array_msg);
}


/**
 * Callback handling dyamic reconfigure
 * @param config
 * @param level
 */
void LayoutManager::reconfigureCallback(road_layout_estimation::road_layout_estimationConfig &config, uint32_t level)
{
    cout << "   Reconfigure callback. Current layout size: " << current_layout.size() << endl;
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

    // update particle-set number (only if this isn't the first run) ------------------------------------------------
//    LayoutManager::first_run = false;
    if(!LayoutManager::first_run){
        if(config.particles_number > num_particles)
        {
            // let's add some empty particles to particle-set:
           int counter = current_layout.size(); //this will keep track of current ID
           int particles_to_add = config.particles_number - num_particles;
           for(int i=0; i<particles_to_add; i++)
           {
//               VectorXd p_pose = VectorXd::Zero(12);
//               MatrixXd p_sigma = MatrixXd::Zero(12,12);
//               Particle part(counter, p_pose, p_sigma, mtn_model);
               Particle part(counter, mtn_model);
               State6DOF tmp;
               tmp.addNoise( 0.05, 0.05, 0.05, 0.05);
               part.setParticleState(tmp);
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

    /// check this flag if we want to initialize particle-set with OSM and GPS
    bool gps_initialization = true;

    if(LayoutManager::first_run){

        if(gps_initialization)
        {
            ROS_INFO_STREAM("Road layout manager first run, init particle-set from GPS signal");

            // -------- WAIT FOR GPS MESSAGE ----------------------------------------- //

//            // wait for GPS message (coming from Android device)
//            sensor_msgs::NavSatFix::ConstPtr gps_msg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/fix");

//            // Save values into local vars (this is a trick when GPS device isn't available or we want to simulate a msg)
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

            // Publish GPS fix on map
//            fix.header.frame_id = "map";
//            fix.header.stamp = ros::Time::now();
//            fix.pose.position.x = latlon_2_xy_srv.response.x;
//            fix.pose.position.y = latlon_2_xy_srv.response.y;
//            fix.pose.orientation.w = 1;
//            LayoutManager::gps_pub.publish(fix);
            // ------------------------ END OF GPS MSG ---------------------------------- //


//            /* ------------------------ WAIT FOR RVIZ INITIAL POSE  --------------------- */
//            geometry_msgs::PoseWithCovarianceStamped::ConstPtr rviz_msg = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/initialpose");

//            // transform coordinates from 'local_map' to 'map'
//            osm_cartography::local_map_transform local_map_tf_srv;
//            double map_x; double map_y;
//            if (LayoutManager::local_map_tf_client.call(local_map_tf_srv))
//            {
//                 map_x = rviz_msg->pose.pose.position.x + local_map_tf_srv.response.tf_x;
//                 map_y = rviz_msg->pose.pose.position.y + local_map_tf_srv.response.tf_y;
//            }
//            else
//            {
//              ROS_ERROR("   Failed to call 'local_map_transform' service");
//              return;
//            }

//            // convert UTM to LatLon
//            osm_cartography::xy_2_latlon xy_2_latlon_srv;
//            xy_2_latlon_srv.request.x = map_x;
//            xy_2_latlon_srv.request.y = map_y;
//            if (!LayoutManager::xy_2_latlon_client.call(xy_2_latlon_srv)){
//                ROS_ERROR("   Failed to call 'xy_2_latlon' service");
//                return;
//            }

//            double alt = 0;
//            double lat = xy_2_latlon_srv.response.latitude;
//            double lon = xy_2_latlon_srv.response.longitude;
//            double cov1 = 15;
//            double cov2 = 15;
//            /* ------------------------ END RVIZ INITIAL POSE  ------------------------- */

            // Get XY values from GPS coords
            osm_cartography::latlon_2_xy latlon_2_xy_srv;
            latlon_2_xy_srv.request.latitude = lat;
            latlon_2_xy_srv.request.longitude = lon;
            geometry_msgs::Point point;
            geometry_msgs::PoseStamped fix;
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
            cout << "   mean: " << endl;
            cout << mean << endl;
            cout << "   cov: " << endl;
            cout << covar << endl;
            cout << "------------------------------------------------------------" << endl << endl;

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
                    cout << "while ctr reached max limit" << endl;
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
                srv.request.max_distance_radius = 200;

                // Call snap particle service service
                if (LayoutManager::snap_particle_xy_client.call(srv))
                {
                    // Init particle's pose
                    State6DOF p_pose;
                    p_pose._pose = Vector3d(srv.response.snapped_x, srv.response.snapped_y, 0);

                    // Create rotation object
                    tf::Quaternion q = tf::createQuaternionFromYaw(srv.response.way_dir_rad);
                    AngleAxisd rotation = AngleAxisd( Quaterniond(q.getW(), q.getX(), q.getY(), q.getZ()) );
                    p_pose.setRotation(rotation);

                    // Init particle's sigma
                    MatrixXd p_sigma = MatrixXd::Zero(12,12);

                    // Push particle into particle-set
                    Particle part(particle_id, p_pose, p_sigma, mtn_model);
                    current_layout.push_back(part);

                    // Update particles id counter
                    particle_id += 1;

                    // Check if we should create 2 particles with opposite direction
                    if(srv.response.way_dir_opposite_particles){

                        // Invert Yaw direction
                        double inverted_angle_rad = Utils::normalize_angle(srv.response.way_dir_rad + M_PI);
                        tf::Quaternion q = tf::createQuaternionFromYaw(inverted_angle_rad);
                        AngleAxisd rotation = AngleAxisd( Quaterniond(q.getW(), q.getX(), q.getY(), q.getZ()) );
                        p_pose.setRotation(rotation);

                        // Push particle inside particle-set
                        Particle opposite_part(particle_id, p_pose, p_sigma, mtn_model);
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
    if(first_msg && current_layout.size() == 0){
        cout << "   First Odometry message arrived, generating particles set" << endl;

        //clear current layout
        current_layout.clear();

        // generate particle set
        for(int i=0; i<num_particles; i++)
        {
            // init an empty particle
            State6DOF p_pose;
            MatrixXd p_sigma = MatrixXd::Zero(12,12);
            Particle part(i, p_pose, p_sigma, mtn_model);

            // get pose & cov from odom msg
            State6DOF new_pose(msg);
            MatrixXd new_cov = Utils::getCovFromOdom(msg);

            // add some random offset to particles (only the first one won't be noised)
            if(i!=1)
            {
                new_pose.addNoise(0.05, 0.05, 0.05, 0.05);
            }

            // update particle values
            part.setParticleState(new_pose);
            part.setParticleSigma(new_cov);

//            // let's add a sample component in same position of the particle
//            layout_components.at(0)->particle_id = part.getId();
//            layout_components.at(0)->component_id = 0;
//            layout_components.at(0)->component_state = new_pose;
//            layout_components.at(0)->component_cov = p_sigma;
//            part.addComponent(layout_components.at(0));

//            layout_components.at(1)->particle_id = part.getId();
//            layout_components.at(1)->component_id = 1;
//            layout_components.at(1)->component_state = new_pose;
//            layout_components.at(1)->component_cov = p_sigma;
//            part.addComponent(layout_components.at(1));

//            layout_components.at(2)->particle_id = part.getId();
//            layout_components.at(2)->component_id = 2;
//            layout_components.at(2)->component_state = new_pose;
//            layout_components.at(2)->component_cov = p_sigma;
//            part.addComponent(layout_components.at(2));

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


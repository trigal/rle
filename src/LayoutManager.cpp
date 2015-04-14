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
#include "LayoutManager.h"

bool LayoutManager::openstreetmap_enabled = true; /// check this flag if we want to initialize particle-set with OSM and GPS
double LayoutManager::delta_t = 1;      /// Initialize static member value for C++ compilation
bool LayoutManager::first_run = true;   /// flag used for initiliazing particle-set with gps
bool LayoutManager::first_msg = true;   /// first odometry msg flag
int LayoutManager::step = 0;            /// filter step counter

visualization_msgs::Marker marker1;
visualization_msgs::Marker marker2;


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
    odometry_sub = node_handle.subscribe(topic, 1, &LayoutManager::odometryCallback, this);
    ROS_INFO_STREAM("ROAD LAYOUT ESTIMATION STARTED, LISTENING TO: " << odometry_sub.getTopic());

    // init road_lane_detection subscriber
    road_lane_sub = node_handle.subscribe("/road_lane_detection/lanes", 3, &LayoutManager::roadLaneCallback, this);

    // init output files
//    stat_out_file.open("/home/limongi/Desktop/culo/output.txt");
    vo_distance_out_file.open("/home/limongi/Desktop/vo_distance.txt"); //ros::package::getPath("road_layout_estimation")
    vo_mm_distance_out_file.open("/home/limongi/Desktop/vo_mm_distance.txt");

    // init values
    step = 0;
    num_particles = 0;
    resampling_count = 0;
    LayoutManager::first_msg = true;

    // init motion model
//    mtn_model = new MotionModel();
    measurement_model = new MeasurementModel();

    // init header timestamp
    old_msg.header.stamp = ros::Time::now();

    // init publisher
    LayoutManager::array_pub = n.advertise<geometry_msgs::PoseArray>("/road_layout_estimation/layout_manager/particle_pose_array",1);
    LayoutManager::gps_pub = n.advertise<geometry_msgs::PoseStamped>("/road_layout_estimation/layout_manager/gps_fix",1);
    LayoutManager::street_publisher = n.advertise<geometry_msgs::PoseStamped> ("/road_layout_estimation/layout_manager/quaternion_pose",1);
    LayoutManager::particle_publisher = n.advertise<geometry_msgs::PoseStamped> ("/road_layout_estimation/layout_manager/particle_pose",1);
    LayoutManager::diff_publisher = n.advertise<geometry_msgs::PoseStamped> ("/road_layout_estimation/layout_manager/diff_pose",1);
    LayoutManager::marker_pub = n.advertise<visualization_msgs::Marker>("/road_layout_estimation/layout_manager/circle", 1);
    LayoutManager::marker_pub2 = n.advertise<visualization_msgs::Marker>("/road_layout_estimation/layout_manager/circle2", 1);
    LayoutManager::publisher_marker_array = n.advertise<visualization_msgs::MarkerArray>("/road_layout_estimation/layout_manager/marker_array", 1);
    LayoutManager::publisher_marker_array_distances = n.advertise<visualization_msgs::MarkerArray>("/road_layout_estimation/layout_manager/marker_array_distances", 1);
    LayoutManager::publisher_marker_array_angles = n.advertise<visualization_msgs::MarkerArray>("/road_layout_estimation/layout_manager/publisher_marker_array_angles", 1);
    LayoutManager::publisher_z_particle = n.advertise<visualization_msgs::MarkerArray>("/road_layout_estimation/layout_manager/z_particle", 1);
    LayoutManager::publisher_z_snapped = n.advertise<visualization_msgs::MarkerArray>("/road_layout_estimation/layout_manager/z_snapped", 1);

    LayoutManager::average_pose = n.advertise<nav_msgs::Odometry>("/road_layout_estimation/layout_manager/average_pose",1);

    // init ROS service client
    latlon_2_xy_client = n.serviceClient<ira_open_street_map::latlon_2_xy>("/ira_open_street_map/latlon_2_xy");
    xy_2_latlon_client = n.serviceClient<ira_open_street_map::xy_2_latlon>("/ira_open_street_map/xy_2_latlon");
    snap_particle_xy_client = n.serviceClient<ira_open_street_map::snap_particle_xy>("/ira_open_street_map/snap_particle_xy");
    get_closest_way_distance_utm_client = n.serviceClient<ira_open_street_map::get_closest_way_distance_utm>("/ira_open_street_map/get_closest_way_distance_utm");

    latlon_2_xy_client.waitForExistence(); // WAIT FOR SERVICE -- the function prints some pretty comments

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

    // update score guassian distribution values
    street_distribution_sigma = config.street_distribution_sigma;
    angle_distribution_sigma = config.angle_distribution_sigma;
    street_distribution_weight = config.street_distribution_weight;
    angle_distribution_weight = config.angle_distribution_weight;


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
                   try{
                        tf_listener.transformPose("map", ros::Time(0), tf_pose_local_map_frame, "local_map", tf_pose_map_frame);
                   }
                   catch (tf::TransformException &ex)
                   {
                       ROS_ERROR("%s",ex.what());
                       ROS_INFO_STREAM("if(!LayoutManager::first_run){ if(config.particles_number > num_particles)");
                       ros::shutdown();
                       return;
                   }

                   // Build request
                   ira_open_street_map::snap_particle_xy srv;
                   srv.request.x = tf_pose_map_frame.getOrigin().getX();
                   srv.request.y = tf_pose_map_frame.getOrigin().getY();
                   srv.request.max_distance_radius = 100; // distance radius for finding the closest nodes for particle snap

                   // Get distance from closest street and set it as particle score
                   // init normal distribution
                   boost::math::normal normal_dist(0, street_distribution_sigma);
                   if (LayoutManager::snap_particle_xy_client.call(srv))
                   {
                       // calculate difference between original particle position and snapped particle position
                       // use it to set particle score using normal distribution PDF
                       double dx = tf_pose_map_frame.getOrigin().getX() - srv.response.snapped_x;
                       double dy = tf_pose_map_frame.getOrigin().getY() - srv.response.snapped_y;
                       double dz = tf_pose_map_frame.getOrigin().getZ() - 0;
                       double distance = sqrt(dx*dx + dy*dy + dz*dz);
                       part.setParticleScore(pdf(normal_dist, distance));
                   }
                   else
                   {
                       // Either service is down or particle is too far from a street
                       part.setParticleScore(0);
                   }
               }

               // Push particle
               current_layout.push_back(part);

               // Update counter
               counter = counter+1;
           }// end cycle for creating particle (FOR)
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

//            // KITTI 00 [OK, si impianta dopo un pò per i ritardi accumulati]
//            double alt = 0;
//            double lat = 48.98254523586602;
//            double lon = 8.39036610004500;
//            double cov1 = 100;
//            double cov2 = 100;

//            // KITTI 01 [OK, video autostrada, si perde nella curva finale]
//            double alt = 0;
//            double lat = 49.006719195871;//49.006558;// 49.006616;//
//            double lon = 8.4893558806503;//8.489195;//8.489291;//
//            double cov1 = 50;
//            double cov2 = 50;

            // KITTI 02 [NI, si perde dopo un paio di curve]
//            double alt = 0;
//            double lat = 48.987607723096;
//            double lon = 8.4697469732634;
//            double cov1 = 60;
//            double cov2 = 60;

//            // KITTI 04 [OK, video road, rettilineo corto]
//            double alt = 0;
//            double lat = 49.033603440345;
//            double lon = 8.3950031909457;
//            double cov1 = 50;
//            double cov2 = 50;

            // KITTI 05 [NI, se imbocca la strada giusta nell'inizializzazione funziona bene]
//            double lat = 49.050384;//49.04999;//49.04951961077; //49.049695;//
//            double lon = 8.396351;// 8.39645;//8.3965961639946; //8.397790;////
            double lat = 49.04951961077;
            double lon = 8.3965961639946;
            double alt = 0;
            double cov1 = 10;
            double cov2 = 10;

            // KITTI 06 [OK, video loop, si perde dopo il secondo incrocio]
//            double alt = 0;
//            double lat = 49.05349304789598;
//            double lon = 8.39721998765449;
//            double cov1 = 50;
//            double cov2 = 50;

            // KITTI 07 [CUTTED OK, video in cui sta fermo allo stop alcuni secondi]
//            double alt = 0;
//            double lat = 48.985319;//48.98523696217;
//            double lon = 8.393801;//8.3936414564418;
//            double cov1 = 50;
//            double cov2 = 50;

            // KITTI 08 [bag inizia dopo]
//            double alt = 0;
//            double lat = 48.984311;
//            double lon = 8.397817;
//            double cov1 = 60;
//            double cov2 = 60;

            // KITTI 09 [OK, video serie curve tondeggianti]
//            double alt = 0;
//            double lat = 48.972104544468;
//            double lon = 8.4761469953335;
//            double cov1 = 60;
//            double cov2 = 60;

            // KITTI 10 [CUTTED OK, non esegue l'inversione finale rimane indietro]
//            double alt = 0;
//            double lat = 48.972406;//48.972455;//48.97253396005;
//            double lon = 8.478662;//8.478660;//8.4785980847297;
//            double cov1 = 50;
//            double cov2 = 50;

            //ros::Duration(1).sleep(); // sleep for 2 secs
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
//            ira_open_street_map::xy_2_latlon xy_2_latlon_srv;
//            xy_2_latlon_srv.request.x = tf_pose_map_frame.getOrigin().getX();
//            xy_2_latlon_srv.request.y = tf_pose_map_frame.getOrigin().getY();
//            if (!LayoutManager::xy_2_latlon_client.call(xy_2_latlon_srv)){
//                ROS_ERROR("   Failed to call 'xy_2_latlon' service");
//                ros::shutdown(); //augusto debug
//                return;
//            }

//            double alt = 0;
//            double lat = xy_2_latlon_srv.response.latitude;
//            double lon = xy_2_latlon_srv.response.longitude;
//            double cov1 = 150;
//            double cov2 = 150;
            // ------------------------ END RVIZ INITIAL POSE  ------------------------- //

            // Get XY values from GPS coords
            ira_open_street_map::latlon_2_xy latlon_2_xy_srv;
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
              ros::shutdown(); //augusto debug
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


            /// ---------------------------------------------------------------------------
            // Set our shape type to be a sphere
            uint32_t shape = visualization_msgs::Marker::SPHERE;

            visualization_msgs::Marker marker;
            // Set the frame ID and timestamp.
            marker.header.frame_id = "map";

            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            marker.ns = "gauss_sigma";
            marker.id = 0;
            marker.type = shape;
            // Set the marker action.  Options are ADD and DELETE
            marker.action = visualization_msgs::Marker::ADD;

            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            marker.pose.position.x = point.x;
            marker.pose.position.y = point.y;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = sqrt(cov1)*3*2; //*2 Scale of the marker. Applied before the position/orientation. A scale of [1,1,1] means the object will be 1m by 1m by 1m.
            marker.scale.y = sqrt(cov2)*3*2; //*2 Scale of the marker. Applied before the position/orientation. A scale of [1,1,1] means the object will be 1m by 1m by 1m.
            marker.scale.z = .1;

            // Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.3f;

            marker.lifetime = ros::Duration();
            marker.header.stamp = ros::Time::now();


            visualization_msgs::Marker marker2;
            // Set the frame ID and timestamp.
            marker2.header.frame_id = "map";

            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            marker2.ns = "gauss_sigma";
            marker2.id = 1;
            marker2.type = shape;
            // Set the marker action.  Options are ADD and DELETE
            marker2.action = visualization_msgs::Marker::ADD;

            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            marker2.pose.position.x = point.x;
            marker2.pose.position.y = point.y;
            marker2.pose.position.z = 0;
            marker2.pose.orientation.x = 0.0;
            marker2.pose.orientation.y = 0.0;
            marker2.pose.orientation.z = 0.0;
            marker2.pose.orientation.w = 1.0;

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker2.scale.x = .1;
            marker2.scale.y = .5;
            marker2.scale.z = 80;

            // Set the color -- be sure to set alpha to something non-zero!
            marker2.color.r = 0.0f;
            marker2.color.g = 0.0f;
            marker2.color.b = 1.0f;
            marker2.color.a = 1.0f;

            marker2.lifetime = ros::Duration();
            marker2.header.stamp = ros::Time::now();

            // Publish the marker
            marker_pub.publish(marker);
            marker_pub2.publish(marker2);

            marker1 = marker;
            marker2 = marker2;
            // -------------------------------------------------------------------------------

            particle_poses_statistics = MatrixXd(config.particles_number,2);

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
                    ros::shutdown(); // augusto debug
                    /**
                     * TODO: max_radius_size * 2 and find again
                     */
                }
                // Generate a sample from the bivariate Gaussian distribution
                Matrix<double,2,-1> sample = normX.samples(1);


                //-------------- SNAP PARTICLE v2  -------------------------------------- //
                // Init OSM cartography service
                ira_open_street_map::snap_particle_xy srv;
                srv.request.x = sample(0);
                srv.request.y = sample(1);
                srv.request.max_distance_radius = 200; // distance radius for finding the closest nodes for particle snap
                boost::math::normal street_normal_dist(0, street_distribution_sigma);
                boost::math::normal angle_normal_dist(0, angle_distribution_sigma);

                // Call snap particle service
                if (LayoutManager::snap_particle_xy_client.call(srv))
                {
                    geometry_msgs::PoseStamped pose_map_frame;
                    pose_map_frame.header.frame_id = "map";
                    pose_map_frame.header.stamp = ros::Time::now();
                    pose_map_frame.pose.position.x = srv.response.snapped_x;
                    pose_map_frame.pose.position.y = srv.response.snapped_y;
                    pose_map_frame.pose.position.z = 0.0;

                    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(srv.response.way_dir_rad), pose_map_frame.pose.orientation);
                    tf::Stamped<tf::Pose> tf_pose_map_frame, tf_pose_local_map_frame;
                    tf::poseStampedMsgToTF(pose_map_frame,tf_pose_map_frame);

                    // Transform pose from "map" to "local_map"
                    try
                    {
                        tf_listener.waitForTransform("local_map", "map", ros::Time(0), ros::Duration(0.5));
                        tf_listener.transformPose("/local_map", ros::Time(0), tf_pose_map_frame, "/map", tf_pose_local_map_frame);
                    }
                    catch (tf::TransformException &ex)
                    {
                        ROS_ERROR("%s",ex.what());
                        ROS_INFO_STREAM("map to local map exception");
                        continue; //Skip this iteration
                    }

                    // Init particle's pose
                    State6DOF p_pose;
                    p_pose._pose = Vector3d(tf_pose_local_map_frame.getOrigin().getX(), tf_pose_local_map_frame.getOrigin().getY(), 0);
                    tf_pose_local_map_frame.setRotation(tf_pose_local_map_frame.getRotation().normalized());

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

                    cout << "p_sigma" << endl << endl << p_sigma << endl;
                    // Create particle and set its score
                    Particle part(particle_id, p_pose, p_sigma, mtn_model);
                    part.setParticleScore(pdf(street_normal_dist,0) + pdf(angle_normal_dist, 0)); // dont' calculate score with distance because particle is snapped

                    // Push particle into particle-set
                    current_layout.push_back(part);

                    // Update particles id counter
                    particle_id += 1;

                    // Check if we should create 2 particles with opposite direction
                    if(srv.response.way_dir_opposite_particles){

                        tf::Stamped<tf::Pose>  tf_pose_local_map_frame_opposite_direction;

                        tf_pose_local_map_frame_opposite_direction = tf_pose_local_map_frame;
                        tf_pose_local_map_frame_opposite_direction.setRotation(tf_pose_local_map_frame*tf::createQuaternionFromYaw(M_PI)); // INVERT DIRECTION

                        AngleAxisd rotation_opposite = AngleAxisd(
                                    Quaterniond(
                                        tf_pose_local_map_frame_opposite_direction.getRotation().getW(),
                                        tf_pose_local_map_frame_opposite_direction.getRotation().getX(),
                                        tf_pose_local_map_frame_opposite_direction.getRotation().getY(),
                                        tf_pose_local_map_frame_opposite_direction.getRotation().getZ()
                                        ));

                        //                        // Invert Yaw direction
                        //                        double angle_temp = M_PI + part.getParticleState().getRotation().angle();
                        //                        cout << "inverse angle: " << angle_temp << endl;
                        //                        cout << "        angle: " << part.getParticleState().getRotation().angle() << endl;

                        p_pose.setRotation(rotation_opposite);

                        // Create particle and set its score
                        Particle opposite_part(particle_id, p_pose, p_sigma, mtn_model);
                        opposite_part.setParticleScore(pdf(street_normal_dist,0) * pdf(angle_normal_dist, 0)); // don't calculate score with distance because particle is snapped

                        // Push particle inside particle-set
                        current_layout.push_back(opposite_part);

                        // Update particles id counter
                        particle_id += 1;
                    }
                }
                else
                {
                    ROS_ERROR("     Failed to call 'snap_particle_xy' service");
                    ros::shutdown(); //augusto debug
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
            ostringstream tmp_convert;   // stream used for the conversion
            tmp_convert << "Particle ID: " << p.getId();
            p.getParticleState().printState(tmp_convert.str());
        }

        // BUILD POSEARRAY MSG
        // Get particle-set
        geometry_msgs::PoseArray array_msg = LayoutManager::buildPoseArrayMsg(particles);
        array_msg.header.stamp = ros::Time::now();
        cout << "poses size: " << array_msg.poses.size() << endl;
        array_msg.header.frame_id = "local_map";
        cout << "frame id: " << array_msg.header.frame_id.c_str() << endl;
        // Publish it!
        LayoutManager::array_pub.publish(array_msg);


        LayoutManager::publishMarkerArray();


    } // end if(first_run)
}// end reconfigure callback


/** *************************************************************************************************
 * Callback called on nav_msg::Odometry arrival
 * @param msg
 ***************************************************************************************************/
void LayoutManager::publishMarkerArray()
{

    //normalizeParticleSet();

    //the same but with marker-array
    marker_array.markers.clear();
    for(int i = 0; i<current_layout.size(); i++)
    {
        Particle p = current_layout.at(i);
        geometry_msgs::Pose pose = p.getParticleState().toGeometryMsgPose();

        tf::Quaternion q;
        tf::quaternionMsgToTF(pose.orientation,q);
        q = q.normalize();
        tf::quaternionTFToMsg(q, pose.orientation);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "local_map";
        marker.header.stamp = ros::Time();
        marker.ns = "cicciobello";
        marker.id = p.getId();
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pose.position.x;
        marker.pose.position.y = pose.position.y;
        marker.pose.position.z = pose.position.z;
        marker.pose.orientation.x = pose.orientation.x;
        marker.pose.orientation.y = pose.orientation.y;
        marker.pose.orientation.z = pose.orientation.z;
        marker.pose.orientation.w = pose.orientation.w;
        marker.scale.x = 5;
        marker.scale.y = 0.5;
        marker.scale.z = 1;
        marker.color.a = p.getParticleScore()+0.5;
        marker.color.r = 0;
        marker.color.g = p.getParticleScore();
        marker.color.b = 0;

        marker_array.markers.push_back(marker);

        //cout << "p.getParticleScore()\t" << p.getParticleScore() << endl;
    }

    publisher_marker_array.publish(marker_array);
}

/**
 * @brief LayoutManager::publishMarkerArrayDistances
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 */
void LayoutManager::publishMarkerArrayDistances(int id, double x1, double y1, double x2, double y2, double z)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "local_map";
    line_list.header.stamp = ros::Time();
    line_list.ns = "lines";
    line_list.id = id;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1;
    line_list.scale.x = 0.2; //width
    line_list.color.a = 1.0;
    line_list.color.r = 0;
    line_list.color.g = 0;
    line_list.color.b = 1.0;

    // Create the points
    geometry_msgs::Point point;
    point.x = x1;
    point.y = y1;
    point.z = z;
    line_list.points.push_back(point);

    point.x = x2;
    point.y = y2;
    point.z = 0;
    line_list.points.push_back(point);

    // Push back line_list
    marker_array_distances.markers.push_back(line_list);
}

/**
 * @brief LayoutManager::publishMarkerArrayDistances
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 */
void LayoutManager::publishZSnapped(int id, double x1, double y1, double x2, double y2, double z)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "local_map";
    line_list.header.stamp = ros::Time();
    line_list.ns = "lines";
    line_list.id = id;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1;
    line_list.scale.x = 0.2; //width
    line_list.color.a = 1.0;
    line_list.color.r = 1.0;
    line_list.color.g = 0;
    line_list.color.b = 0.0;

    // Create the points
    geometry_msgs::Point point;
    point.x = x1;
    point.y = y1;
    point.z = z;
    line_list.points.push_back(point);

    point.x = x2;
    point.y = y2;
    point.z = 0;
    line_list.points.push_back(point);

    // Push back line_list
    marker_z_snapped.markers.push_back(line_list);
}

/**
 * @brief LayoutManager::publishMarkerArrayDistances
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 */
void LayoutManager::publishZParticle(int id, double x1, double y1, double x2, double y2, double z)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "local_map";
    line_list.header.stamp = ros::Time();
    line_list.ns = "lines";
    line_list.id = id;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1;
    line_list.scale.x = 0.2; //width
    line_list.color.a = 1.0;
    line_list.color.r = 0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;

    // Create the points
    geometry_msgs::Point point;
    point.x = x1;
    point.y = y1;
    point.z = z;
    line_list.points.push_back(point);

    point.x = x2;
    point.y = y2;
    point.z = 0;
    line_list.points.push_back(point);

    // Push back line_list
    marker_z_particle.markers.push_back(line_list);
}

void LayoutManager::odometryCallback(const nav_msgs::Odometry& msg)
{
    // Beep
    printf("\a");

    // Publish GPS init markers
    marker_pub.publish(marker1);
    marker_pub2.publish(marker2);

    cout << "--------------------------------------------------------------------------------" << endl;
    cout << "[step: " << step << "]" << endl; step++;

    // stampo misura arrivata
    std::cout << " ******* MSG ARRIVED. *******" << std::endl;
//    Utils::printOdomAngleAxisToCout(msg);

    // if it's our first incoming odometry msg:
    // (OLD)    just use it as particle-set poses initializer
    // (NEW)    filter won't be called
//    if(LayoutManager::first_msg && current_layout.size() == 0){
//        cout << "   First Odometry message arrived, no particles" << endl;
//        return;
//    }

    if(LayoutManager::first_msg)
    {
        old_msg = msg;
    }

    // update flag
    LayoutManager::first_msg = false;

    // retrieve measurement from odometry
//    State6DOF(measurement_model->getOldMsg()).printState("[old_msg]");
    measurement_model->setMsg(msg);

    // calculate delta_t
    delta_t = msg.header.stamp.toSec() - old_msg.header.stamp.toSec();

//    std::cout << "DELTA_T:\n" << msg.header.stamp.toSec() << "\n" << old_msg.header.stamp.toSec() << "\n" << delta_t << endl;

    // Diff quaternions used for angle score
    tf::Quaternion first_quaternion_diff;
    tf::Quaternion second_quaternion_diff;
    tf::Quaternion street_direction;

    // % Blue distance lines marker array
    marker_array_distances.markers.clear();

    if (!measurement_model->isMeasureValid())
    {
        ROS_WARN("LayoutManager says: Invalid measure detected");
    }


    // call particle_estimation -------------------------------------------------------------------------------------------------------
    vector<Particle>::iterator particle_itr;
    for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ ){

        // SAMPLING ---------------------------------------------------------------------------------------------
        // estimate particle
        (*particle_itr).particlePoseEstimation(measurement_model);


        // SCORE ------------------------------------------------------------------------------------------------
        // update particle score using OpenStreetMap
        if(LayoutManager::openstreetmap_enabled)
        {

            //AUGUSTO FOR DEBUGGING
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            tf::Quaternion q;

            // Get particle state
            Vector3d p_state = (*particle_itr).getParticleState().getPose();
            geometry_msgs::PoseStamped pose_local_map_frame;
            pose_local_map_frame.header.frame_id = "local_map";
            pose_local_map_frame.header.stamp = ros::Time::now();
            pose_local_map_frame.pose.position.x = p_state(0);
            pose_local_map_frame.pose.position.y =  p_state(1);
            pose_local_map_frame.pose.position.z =  p_state(2);
            Eigen::Quaterniond tmp_quat((*particle_itr).getParticleState().getRotation());
            pose_local_map_frame.pose.orientation.w = tmp_quat.w();
            pose_local_map_frame.pose.orientation.x = tmp_quat.x();
            pose_local_map_frame.pose.orientation.y = tmp_quat.y();
            pose_local_map_frame.pose.orientation.z = tmp_quat.z();

            tf::Stamped<tf::Pose> tf_pose_map_frame, tf_pose_local_map_frame;
            tf::poseStampedMsgToTF(pose_local_map_frame, tf_pose_local_map_frame);
            // Transform pose from "local_map" to "map"
            try{
                tf_listener.transformPose("map", ros::Time(0), tf_pose_local_map_frame, "local_map", tf_pose_map_frame);

            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ROS_ERROR("     Transform snapped particle pose from local_map to map");
                ros::shutdown();
                return;
            }


//            //TEST 0
            transform.setOrigin( tf::Vector3(pose_local_map_frame.pose.position.x, pose_local_map_frame.pose.position.y, pose_local_map_frame.pose.position.z) );
            q.setX(pose_local_map_frame.pose.orientation.x);
            q.setY(pose_local_map_frame.pose.orientation.y);
            q.setZ(pose_local_map_frame.pose.orientation.z);
            q.setW(pose_local_map_frame.pose.orientation.w);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_map", "pose_local_map_frame"));
            transform.setOrigin( tf_pose_local_map_frame.getOrigin());
            transform.setRotation(tf_pose_local_map_frame.getRotation());
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_map", "tf_pose_local_map_frame"));


            // Build request for getting snapped XY values + orientation of the road
            ira_open_street_map::snap_particle_xy srv;
            srv.request.x = tf_pose_map_frame.getOrigin().getX();
            srv.request.y = tf_pose_map_frame.getOrigin().getY();
            srv.request.max_distance_radius = 100; // distance radius for finding the closest nodes for particle snap


            // Get distance from snapped particle pose and set it as particle score
            if (LayoutManager::snap_particle_xy_client.call(srv))
            {
                // Snapped pose is is map frame, convert from MSG to TF first.
                geometry_msgs::PoseStamped snapped_map_frame;
                snapped_map_frame.header.frame_id = "map";
                snapped_map_frame.header.stamp = ros::Time::now();
                snapped_map_frame.pose.position.x = srv.response.snapped_x;
                snapped_map_frame.pose.position.y =  srv.response.snapped_y;
                snapped_map_frame.pose.position.z =  0;
                snapped_map_frame.pose.orientation.x = srv.response.way_dir_quat_x;
                snapped_map_frame.pose.orientation.y = srv.response.way_dir_quat_y;
                snapped_map_frame.pose.orientation.z = srv.response.way_dir_quat_z;
                snapped_map_frame.pose.orientation.w = srv.response.way_dir_quat_w;

                tf::Stamped<tf::Pose> tf_snapped_map_frame, tf_snapped_local_map_frame, tf_snapped_local_map_frame_opposite_direction ;
                tf::poseStampedMsgToTF(snapped_map_frame, tf_snapped_map_frame);

                //TEST 1 ---- BE CAREFUL, THIS MAY NOT BE THE RIGHT DIRECTION!
                transform.setOrigin( tf::Vector3(snapped_map_frame.pose.position.x, snapped_map_frame.pose.position.y, snapped_map_frame.pose.position.z) );
                q.setX(snapped_map_frame.pose.orientation.x);
                q.setY(snapped_map_frame.pose.orientation.y);
                q.setZ(snapped_map_frame.pose.orientation.z);
                q.setW(snapped_map_frame.pose.orientation.w);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "snapped_map_frame"));
                transform.setOrigin( tf_snapped_map_frame.getOrigin());
                transform.setRotation(tf_snapped_map_frame.getRotation());
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "tf_snapped_map_frame"));

//                //TEST 2. CHECKED LATER IN THE CODE, BUT KNOW BY ME.
//                if (srv.response.way_dir_opposite_particles)
//                {
//                    tf_snapped_map_frame.setRotation(tf_snapped_map_frame*tf::createQuaternionFromYaw(M_PI));

//                    transform.setOrigin( tf_snapped_map_frame.getOrigin());
//                    transform.setRotation(tf_snapped_map_frame.getRotation());
//                    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "tf_snapped_map_frame_INVERTED_DIRECTION"));

//                    // ** THIS ROUTING LEAVES THE ROTATION INVERTED **
//                }

                // Transform pose from "map" to "local_map"
                try{
                    tf_listener.transformPose("local_map", ros::Time(0), tf_snapped_map_frame, "map", tf_snapped_local_map_frame);
                }
                catch (tf::TransformException &ex)
                {
                    ROS_ERROR("%s",ex.what());
                    ROS_ERROR("     Transform snapped particle pose from map to local_map");
                    ros::shutdown();
                    return;
                }

                // TEST 3. ** BE CAREFUL, THIS IS EQUAL TO INVERTED DIRECTION IF TEST2 IS ENABLED .
                transform.setOrigin( tf_snapped_local_map_frame.getOrigin());
                transform.setRotation(tf_snapped_local_map_frame.getRotation());
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_map", "tf_snapped_local_map_frame"));

                // calculate distance from original particle positin and snapped particle position ---------------------------------
                // use it for score calculation with normal distribution PDF
                double dx = tf_pose_local_map_frame.getOrigin().getX() - tf_snapped_local_map_frame.getOrigin().getX();
                double dy = tf_pose_local_map_frame.getOrigin().getY() - tf_snapped_local_map_frame.getOrigin().getY();
                double dz = tf_pose_local_map_frame.getOrigin().getZ() - 0; // particle Z axis is forced to be next to zero
                double distance = sqrt(dx*dx + dy*dy + dz*dz);

                // add line to marker array distances
                publishMarkerArrayDistances((*particle_itr).getId(),
                                            tf_pose_local_map_frame.getOrigin().getX(),
                                            tf_pose_local_map_frame.getOrigin().getY(),
                                            tf_snapped_local_map_frame.getOrigin().getX(),
                                            tf_snapped_local_map_frame.getOrigin().getY(),
                                            tf_pose_local_map_frame.getOrigin().getZ());

                //      get PDF score FOR DISTANCE
                boost::math::normal normal_dist(0, street_distribution_sigma);
                double pose_diff_score_component = pdf(normal_dist, distance);
                //tf_snapped_local_map_frame.getRotation().getAngleShortestPath(tf_pose_local_map_frame.getRotation()) // check this out, maybe works .. this line isn't tested yet

                // calculate angle difference ---------------------------------------------------------------------------------------
                first_quaternion_diff = tf_snapped_local_map_frame.getRotation().inverse() * tf_pose_local_map_frame.getRotation();
                street_direction=first_quaternion_diff;

                //      get PDF score from first angle
                boost::math::normal angle_normal_dist(0, angle_distribution_sigma);
                double first_angle_difference = Utils::normalize_angle(first_quaternion_diff.getAngle());
                double first_angle_diff_score = pdf(angle_normal_dist, first_angle_difference);

                double final_angle_diff_score  = 0.0f;
                double second_angle_diff_score = 0.0f;
                double second_angle_difference = 0.0f;

                //      if street have 2 directions check angle diff with opposite angle
                if(srv.response.way_dir_opposite_particles)
                {
                    tf_snapped_local_map_frame_opposite_direction=tf_snapped_local_map_frame; // COPY TRANSFORM (I PRAY FOR THIS)
                    tf_snapped_local_map_frame_opposite_direction.setRotation(tf_snapped_local_map_frame*tf::createQuaternionFromYaw(M_PI)); // INVERT DIRECTION
                    second_quaternion_diff = tf_snapped_local_map_frame_opposite_direction.getRotation().inverse() * tf_pose_local_map_frame.getRotation();

                    //      get PDF score
                    second_angle_difference = Utils::normalize_angle(second_quaternion_diff.getAngle());
                    second_angle_diff_score = pdf(angle_normal_dist, second_angle_difference);

                    //      set score
                    if(second_angle_diff_score > first_angle_diff_score)
                        final_angle_diff_score = second_angle_diff_score;
                    else
                        final_angle_diff_score = first_angle_diff_score;

//                    ROS_INFO_STREAM("ONEWAY-N\tSPATIAL_DISTANCE: "<< distance << "\tANGLE_1: " << first_angle_difference << "\tANGLE_2: " << second_angle_difference);
                }
                else
                {
//                    ROS_INFO_STREAM("ONEWAY-Y\tSPATIAL_DISTANCE: "<< distance << "\tANGLE_1: " << first_angle_difference);
                    final_angle_diff_score = first_angle_diff_score;
                }

                // TEST FINAL. ** BE CAREFUL, THIS IS EQUAL TO INVERTED DIRECTION IF TEST2 IS ENABLED .
                transform.setOrigin( tf_pose_local_map_frame.getOrigin());
                transform.setRotation(tf_pose_local_map_frame.getRotation()*street_direction.inverse());
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_map", "tf_pose_local_map_frame_ROTATED"));

                // set particle score
//                (*particle_itr).setParticleScore(street_distribution_weight/tot_weight * pose_diff_score_component + angle_distribution_weight/tot_weight * angle_diff_score_component);
                (*particle_itr).setParticleScore(street_distribution_weight * pose_diff_score_component * angle_distribution_weight * final_angle_diff_score);


//                cout << std::setprecision(5) << "PARTICLE ID: " << (*particle_itr).getId() << endl
//                     << "  SCORE:" << endl
//                     << "   DISTANCE:" << endl
//                     << "      sigma: " << street_distribution_sigma << endl
//                     << "      error: " << distance << endl
//                     << "      score: " << pose_diff_score_component << endl
//                     << "   ANGLE:" << endl
//                     << "      sigma: " << angle_distribution_sigma << endl
//                     << "     error1: " << first_angle_difference << " \tscore: " << first_angle_diff_score << endl
//                     << "     error2: " << second_angle_difference <<" \tscore: " << second_angle_diff_score <<  endl
//                     << "  sel error: " << street_direction.inverse().getAngle()<< endl
//                     << "      score: " << final_angle_diff_score << endl
//                     << "FINAL SCORE: " << (*particle_itr).getParticleScore() << endl;

                // DARIO
//                cout << "PARTICLE ID: " << (*particle_itr).getId() << endl
//                     << "  SCORE:" << endl
//                     << "   DISTANCE:" << endl
//                     << "      sigma: " << street_distribution_sigma << endl
//                     << "     weight: " << street_distribution_weight << endl
//                     << "      error: " << distance << endl
//                     << "      score: " << pose_diff_score_component << endl
//                     << "   ANGLE:" << endl
//                     << "      sigma: " << angle_distribution_sigma << endl
//                     << "     weight: " << angle_distribution_weight << endl
//                     << "     error1: " << first_angle_difference << endl
//                     << "     score1: " << first_angle_diff_score << endl
//                     << "     error2: " << second_angle_difference << endl
//                     << "     score2: " << second_angle_diff_score << endl
//                     << "      score: " << final_angle_diff_score << endl
//                     << "FINAL SCORE: " << (*particle_itr).getParticleScore() << endl;
            }
            else
            {
                // Either service is down or particle is too far from a street
                (*particle_itr).setParticleScore(0);
                ros::shutdown();
            }// end snap particle client
        } // end openstreetmap enabled
    } // end particle cycle

    // % LINES
    publisher_marker_array_distances.publish(marker_array_distances);

    // % Z
    publisher_z_snapped.publish(marker_z_snapped);
    publisher_z_particle.publish(marker_z_particle);
    // -------------------------------------------------------------------------------------------------------------------------------------

    // RESAMPLING --------------------------------------------------------------------------------------------------------------------------
    if(resampling_count++ == 7)
//    if(0)
    {
        resampling_count = 0;
        vector<Particle> new_current_layout;
        vector<double> particle_score_vect;
        double cum_score_sum = 0;

        // Build cumulative sum of the score and roulette vector
        for(particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ )
        {
             cum_score_sum += (*particle_itr).getParticleScore();
             particle_score_vect.push_back(cum_score_sum);
        }


        if(cum_score_sum != 0)
        {
            // Init uniform distribution
            boost::uniform_real<double> uniform_rand(0, cum_score_sum);
            boost::uniform_real<double> uniform_rand2(0, 100);
            boost::uniform_real<double> uniform_rand3(0, current_layout.size());

            // find weight that is at least num
            vector<double>::iterator score_itr;

            for(int k = 0; k<current_layout.size(); ++k)
            {
                if(uniform_rand2(rng) <= 80) //This percentage of weighted samples
                {
                    // WEIGHTED RESAMPLER
                    int particle_counter = 0;
                    double num = uniform_rand(rng);

                    for(score_itr = particle_score_vect.begin(); score_itr != particle_score_vect.end(); score_itr++ )
                    {
                        if( *score_itr >= num)
                        {
                            Particle temp_part = current_layout.at(particle_counter);
                            temp_part.setId(k);
//                            stat_out_file << temp_part.getId() << "\t";
//                            particle_poses_statistics(k,0) = (temp_part.getParticleState().getPose())(0);
//                            particle_poses_statistics(k,1) = (temp_part.getParticleState().getPose())(1);
                            new_current_layout.push_back(temp_part);
                            break;
                        }
                        particle_counter++;
                    }
                }
                else
                {
                    // UNIFORM RESAMPLER
                    int temp_rand = floor(uniform_rand3(rng));
                    Particle temp_part = current_layout.at(temp_rand);
                    temp_part.setId(k);
                    new_current_layout.push_back(temp_part);
                }

            }

            // Save stats to file
//            MatrixXd centered = particle_poses_statistics.rowwise() - particle_poses_statistics.colwise().mean();
//            MatrixXd cov = (centered.adjoint() * centered) / double(particle_poses_statistics.rows() - 1);
//            double var_mean = 3*sqrt(0.5 * cov(0,0) + 0.5 * cov(1,1));
//            stat_out_file << var_mean << endl;

            // copy resampled particle-set
            current_layout.clear();
            current_layout = new_current_layout;
        }
    }
    // END RESAMPLING ----------------------------------------------------------------------------------------------------------------------

    // -------------------------------------------------------------------------------------------------------------------------------------
    // BUILD POSEARRAY MSG
    // Get particle-set
    vector<Particle> particles = current_layout;
    geometry_msgs::PoseArray array_msg = LayoutManager::buildPoseArrayMsg(particles);
    array_msg.header.stamp = ros::Time::now();//msg.header.stamp;

    // Publish it!
    array_pub.publish(array_msg);
    LayoutManager::publishMarkerArray();
    // -------------------------------------------------------------------------------------------------------------------------------------

    // Update old_msg with current one for next step delta_t calculation
    old_msg = msg;


    // RESULTS
    std::vector<double> scores;
    State6DOF state ;
    Vector3d pose;
    //Eigen::Quaterniond average_quaternion = Eigen::Quaterniond::Identity();
    tf::Quaternion average_quaternion = tf::createIdentityQuaternion();
    average_quaternion.setX(0);
    average_quaternion.setY(0);
    average_quaternion.setZ(0);
    average_quaternion.setW(0);

    double tot_score = 0.0f;
    for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ ){
//        ROS_INFO_STREAM("PARTICLE "<< std::setprecision(5) << (*particle_itr).getId() << "\t" << (*particle_itr).getParticleScore() << "\t" << tot_score);
        tot_score += (*particle_itr).getParticleScore();
    }

    for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ ){
        state = (*particle_itr).getParticleState();

        pose += state.getPose() * (*particle_itr).getParticleScore() / tot_score;

        Eigen::Quaterniond q = Eigen::Quaterniond(state.getRotation());
        tf::Quaternion t;
        t.setX(q.x());
        t.setY(q.y());
        t.setZ(q.z());
        t.setW(q.w());
//        ROS_INFO_STREAM("TF QUAT\t" << t.getX() << " "<< t.getY() << " "<< t.getZ() << " "<< t.getW() << " " << q.vec().transpose());

        average_quaternion += t.slerp(tf::createIdentityQuaternion(),(*particle_itr).getParticleScore() / tot_score);
    }

    average_quaternion.normalize();

    nav_msgs::Odometry odometry;
    odometry.header.frame_id="/local_map";
    odometry.header.stamp=ros::Time::now();

    odometry.pose.pose.position.x=pose(0);
    odometry.pose.pose.position.y=pose(1);
    odometry.pose.pose.position.z=pose(2);
    tf::quaternionTFToMsg(average_quaternion,odometry.pose.pose.orientation);

    average_pose.publish(odometry);

    ROS_INFO_STREAM("AVERAGE SCORE: " << pose.transpose() << "\t" << average_quaternion.getAngle() << "\taverage score: " << tot_score / current_layout.size() << endl);;
    //double sum = std::accumulate(scores.begin(), scores.end(), 0.0);


    // -------------------------------------------------------------------------------------------------------------------------------------
    // SAVE RESULTS TO OUTPUT FILE:
    vo_distance_out_file << "ciao" << "\n";
    vo_mm_distance_out_file << "ciao" << "\n";
    // -------------------------------------------------------------------------------------------------------------------------------------
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
            lc->calculateComponentScore();
        }
    }
}
/** **************************************************************************************************************/

/** **************************************************************************************************************/
/**
 * FORMULA CALCOLO SCORE
 *
 * Cardinalità unaria
 *  NO: 1- Kalman gain sulla pose della particella
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
            (*particle_itr).particlePoseEstimation(measurement_model);
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

void LayoutManager::normalizeParticleSet(){

    vector<Particle>::iterator itr;
    double sum=0.0f;
    double max=-1.0f;
    for (itr = current_layout.begin(); itr != current_layout.end(); itr++)
        if ((*itr).getParticleScore()>max)
            max=(*itr).getParticleScore();
    for (itr = current_layout.begin(); itr != current_layout.end(); itr++)
        (*itr).setParticleScore((*itr).getParticleScore()/max);
}


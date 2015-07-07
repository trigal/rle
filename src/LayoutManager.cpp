/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Authors:                                                              *
 *              Augusto Luis Ballardini - ballardini@disco.unimib.it       *
 *              Axel         Furlan     - furlan@disco.unimib.it           *
 *              Dario        Limongi    - dario.limongi@gmail.com          *
 *                                                                         *
 ***************************************************************************/
#include "LayoutManager.h"

bool    LayoutManager::openstreetmap_enabled = true;     /// Check this flag if we want to initialize particle-set with OSM and GPS
double  LayoutManager::deltaOdomTime = 1;                /// Initialize static member value for C++ compilation
double  LayoutManager::deltaTimerTime = 0.1;             /// Initialize static member value for C++ compilation
bool    LayoutManager::layoutManagerFirstRun = true;     /// Flag used for initiliazing particle-set with gps
bool    LayoutManager::first_msg = true;                 /// First odometry msg flag
int     LayoutManager::odometryMessageCounter = 0;       /// Filter step counter

visualization_msgs::Marker marker1;
visualization_msgs::Marker marker2;

///
/// \brief LayoutManager::buildPoseArrayMsg
///        Creates a geometry message poseArray to visualize the particle pose into RVIZ
///
/// \param particles
/// \return geometry_msgs::PoseArray
///
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

LayoutManager::LayoutManager(ros::NodeHandle& n, std::string& topic, string &bagfile, double timerInterval, ros::console::Level loggingLevel)
{

    /// This sets the logger level; use this to disable all ROS prints
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, loggingLevel) )
        ros::console::notifyLoggerLevelsChanged();
    else
        std::cout << "Error while setting the logger level!" << std::endl;

    /// Init values
    odometryMessageCounter = 0;
    num_particles = 0;
    resampling_count = 0;
    LayoutManager::first_msg = true;
    visualOdometryOldMsg.header.stamp = ros::Time::now();    // init header timestamp
    node_handle = n;                                         // set this node_handle as the same of 'road_layout_manager'
    start_with_gps_message  = true;                          // select RLE mode, hard-coded KITTI initializations, or GPS message TODO:FIX this shame

    this->bagfile = bagfile;

    // init motion model
    // mtn_model = new MotionModel(); TODO: perché qui è stato rimosso? ora dove è?
    // Augusto: è nella reconfigure_callback , appena rinominato come default_motion_model...
    // non capisco perché non è un puntatore ad un oggetto come per il measurement

    measurement_model = new MeasurementModel();


    /// RETRIEVE THE FIXED TRANSFORM BETWEEN ODOM AND VISUAL_ODOMETRY_X_FORWARD
    ///
    /// TODO:  replace hard-coded fixed transform with the following lookuptransorm
    /// Issue: #420

    tf::StampedTransform fixed_transform ;
    fixed_transform.setOrigin(tf::Vector3(0.0f,0.0f,0.0f));
    fixed_transform.setRotation(tf::createQuaternionFromRPY(-1.570796f, 0.0f, -1.570796f ));
    measurement_model->setFixed_transform(fixed_transform);    

    //    try
    //    {
    //        ROS_INFO_STREAM("LayoutManager.cpp Looking up the fixed transform between visual_odometry_odom_x_forward and odom");
    //        tf::StampedTransform fixed_transform;
    //        ros::Time now=ros::Time::now();
    ////        tf_listener.waitForTransform("/visual_odometry_odom_x_forward","/odom",now,ros::Duration(10)); //this is fixed, do not lookup for it every time
    ////        tf_listener.lookupTransform("/visual_odometry_odom_x_forward","/odom",now,fixed_transform);
    //        ROS_INFO_STREAM("OK VOXF to ODOM");
    //    }
    //    catch (tf::TransformException &ex)
    //    {
    //        ROS_ERROR_STREAM("LayoutManager.cpp FAILED to look up the fixed transform between visual_odometry_odom_x_forward and odom" << endl <<ex.what());
    //        this->~LayoutManager();
    //        return;
    //    }


    /// Init publisher & subscribers
    LayoutManager::odometry_sub  = node_handle.subscribe(topic, 1, &LayoutManager::odometryCallback, this);
    LayoutManager::road_lane_sub = node_handle.subscribe("/road_lane_detection/lanes", 3, &LayoutManager::roadLaneCallback , this);
    //LayoutManager::roadState_sub = node_handle.subscribe("/fakeDetector/roadState"   , 3, &LayoutManager::roadStateCallback, this);  //fake detector
    LayoutManager::roadState_sub = node_handle.subscribe("/kitti_player/lanes"   , 3, &LayoutManager::roadStateCallback, this);        //ISISLAB Ruben callback


    ROS_INFO_STREAM("RLE started, listening to: " << odometry_sub.getTopic());

    LayoutManager::array_pub                            = n.advertise<geometry_msgs::PoseArray>              ("/road_layout_estimation/layout_manager/particle_pose_array",1);
    LayoutManager::gps_pub                              = n.advertise<sensor_msgs::NavSatFix>                ("/road_layout_estimation/layout_manager/gps_fix",1);
    LayoutManager::street_publisher                     = n.advertise<geometry_msgs::PoseStamped>            ("/road_layout_estimation/layout_manager/quaternion_pose",1);
    LayoutManager::particle_publisher                   = n.advertise<geometry_msgs::PoseStamped>            ("/road_layout_estimation/layout_manager/particle_pose",1);
    LayoutManager::diff_publisher                       = n.advertise<geometry_msgs::PoseStamped>            ("/road_layout_estimation/layout_manager/diff_pose",1);
    LayoutManager::marker_pub                           = n.advertise<visualization_msgs::Marker>            ("/road_layout_estimation/layout_manager/circle", 1);
    LayoutManager::marker_pub2                          = n.advertise<visualization_msgs::Marker>            ("/road_layout_estimation/layout_manager/circle2", 1);
    LayoutManager::publisher_marker_array               = n.advertise<visualization_msgs::MarkerArray>       ("/road_layout_estimation/layout_manager/particle_set", 1);
    LayoutManager::publisher_marker_array_distances     = n.advertise<visualization_msgs::MarkerArray>       ("/road_layout_estimation/layout_manager/marker_array_distances", 1);
    LayoutManager::publisher_marker_array_angles        = n.advertise<visualization_msgs::MarkerArray>       ("/road_layout_estimation/layout_manager/publisher_marker_array_angles", 1);
    LayoutManager::publisher_z_particle                 = n.advertise<visualization_msgs::MarkerArray>       ("/road_layout_estimation/layout_manager/z_particle", 1);
    LayoutManager::publisher_z_snapped                  = n.advertise<visualization_msgs::MarkerArray>       ("/road_layout_estimation/layout_manager/z_snapped", 1);
    LayoutManager::publisher_GT_RTK                     = n.advertise<visualization_msgs::MarkerArray>       ("/road_layout_estimation/layout_manager/GT_RTK", 1);
    LayoutManager::publisher_average_pose               = n.advertise<nav_msgs::Odometry>                    ("/road_layout_estimation/layout_manager/average_pose",1);

    LayoutManager::publisher_debugInformation           = n.advertise<road_layout_estimation::msg_debugInformation >("/road_layout_estimation/debugInformation",1);

    /// Init ROS service clients
    latlon_2_xy_client                  = n.serviceClient<ira_open_street_map::latlon_2_xy>                  ("/ira_open_street_map/latlon_2_xy");
    xy_2_latlon_client                  = n.serviceClient<ira_open_street_map::xy_2_latlon>                  ("/ira_open_street_map/xy_2_latlon");
    snap_particle_xy_client             = n.serviceClient<ira_open_street_map::snap_particle_xy>             ("/ira_open_street_map/snap_particle_xy");
    get_closest_way_distance_utm_client = n.serviceClient<ira_open_street_map::get_closest_way_distance_utm> ("/ira_open_street_map/get_closest_way_distance_utm");
    getHighwayInfo_client               = n.serviceClient<ira_open_street_map::getHighwayInfo>               ("/ira_open_street_map/getHighwayInfo");
    getDistanceFromLaneCenter_client    = n.serviceClient<ira_open_street_map::getDistanceFromLaneCenter>    ("/ira_open_street_map/getDistanceFromLaneCenter");

    /// Init ROS service server
    server_getAllParticlesLatLon        = n.advertiseService("/road_layout_estimation/layout_manager/getAllParticlesLatLon" , &LayoutManager::getAllParticlesLatLonService, this);


    /// RLE system initialization

    latlon_2_xy_client.waitForExistence();      // WAIT FOR SERVICE -- the function prints some pretty comments

    deltaTimerTime = timerInterval;             //deltaTimer of the LayoutManager Class is initialy set as the requested interval
    RLE_timer_loop = n.createTimer(ros::Duration(deltaTimerTime), &LayoutManager::layoutEstimation, this,false, false);

    /// For debug purposes, print default paramenters (from .launch or .cfg file)
    road_layout_estimation::road_layout_estimationConfig defaultConfigFromLaunchFile;
    this->node_handle.param("propagate_translational_absolute_vel_error_x"  ,            defaultConfigFromLaunchFile.propagate_translational_absolute_vel_error_x   , 0.0);
    this->node_handle.param("propagate_translational_absolute_vel_error_y"  ,            defaultConfigFromLaunchFile.propagate_translational_absolute_vel_error_y   , 0.0);
    this->node_handle.param("propagate_translational_absolute_vel_error_z"  ,            defaultConfigFromLaunchFile.propagate_translational_absolute_vel_error_z   , 0.0);
    this->node_handle.param("propagate_rotational_absolute_vel_error"       ,            defaultConfigFromLaunchFile.propagate_rotational_absolute_vel_error        , 0.0);
    this->node_handle.param("propagate_translational_percentage_vel_error_x",            defaultConfigFromLaunchFile.propagate_translational_percentage_vel_error_x , 0.0);
    this->node_handle.param("propagate_translational_percentage_vel_error_y",            defaultConfigFromLaunchFile.propagate_translational_percentage_vel_error_y , 0.0);
    this->node_handle.param("propagate_translational_percentage_vel_error_z",            defaultConfigFromLaunchFile.propagate_translational_percentage_vel_error_z , 0.0);
    this->node_handle.param("propagate_rotational_percentage_vel_error"     ,            defaultConfigFromLaunchFile.propagate_rotational_percentage_vel_error      , 0.0);
    ROS_DEBUG_STREAM("propagate_translational_absolute_vel_error_x    ------------  " << defaultConfigFromLaunchFile.propagate_translational_absolute_vel_error_x  );
    ROS_DEBUG_STREAM("propagate_translational_absolute_vel_error_y    ------------  " << defaultConfigFromLaunchFile.propagate_translational_absolute_vel_error_y  );
    ROS_DEBUG_STREAM("propagate_translational_absolute_vel_error_z    ------------  " << defaultConfigFromLaunchFile.propagate_translational_absolute_vel_error_z  );
    ROS_DEBUG_STREAM("propagate_rotational_absolute_vel_error         ------------  " << defaultConfigFromLaunchFile.propagate_rotational_absolute_vel_error       );
    ROS_DEBUG_STREAM("propagate_translational_percentage_vel_error_x  ------------  " << defaultConfigFromLaunchFile.propagate_translational_percentage_vel_error_x);
    ROS_DEBUG_STREAM("propagate_translational_percentage_vel_error_y  ------------  " << defaultConfigFromLaunchFile.propagate_translational_percentage_vel_error_y);
    ROS_DEBUG_STREAM("propagate_translational_percentage_vel_error_z  ------------  " << defaultConfigFromLaunchFile.propagate_translational_percentage_vel_error_z);
    ROS_DEBUG_STREAM("propagate_rotational_percentage_vel_error       ------------  " << defaultConfigFromLaunchFile.propagate_rotational_percentage_vel_error     );

    f = boost::bind(&LayoutManager::reconfigureCallback, this, _1, _2);
    server.setCallback(f);
}

void LayoutManager::publish_initial_markers(double cov1, double cov2, geometry_msgs::Point point)
{
    uint32_t shape = visualization_msgs::Marker::SPHERE;

    visualization_msgs::Marker tmp_marker1;
    // Set the frame ID and timestamp.
    tmp_marker1.header.frame_id = "map";

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    tmp_marker1.ns = "gauss_sigma";
    tmp_marker1.id = 0;
    tmp_marker1.type = shape;
    // Set the marker action.  Options are ADD and DELETE
    tmp_marker1.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    tmp_marker1.pose.position.x = point.x;
    tmp_marker1.pose.position.y = point.y;
    tmp_marker1.pose.position.z = 0;
    tmp_marker1.pose.orientation.x = 0.0;
    tmp_marker1.pose.orientation.y = 0.0;
    tmp_marker1.pose.orientation.z = 0.0;
    tmp_marker1.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    tmp_marker1.scale.x = sqrt(cov1)*3*2; //*2 Scale of the marker. Applied before the position/orientation. A scale of [1,1,1] means the object will be 1m by 1m by 1m.
    tmp_marker1.scale.y = sqrt(cov2)*3*2; //*2 Scale of the marker. Applied before the position/orientation. A scale of [1,1,1] means the object will be 1m by 1m by 1m.
    tmp_marker1.scale.z = .1;

    // Set the color -- be sure to set alpha to something non-zero!
    tmp_marker1.color.r = 0.0f;
    tmp_marker1.color.g = 1.0f;
    tmp_marker1.color.b = 0.0f;
    tmp_marker1.color.a = 0.3f;

    tmp_marker1.lifetime = ros::Duration(0); // How long the object should last before being automatically deleted.  0 means forever
    tmp_marker1.header.stamp = ros::Time::now();


    visualization_msgs::Marker tmp_marker2;
    // Set the frame ID and timestamp.
    tmp_marker2.header.frame_id = "map";

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    tmp_marker2.ns = "gauss_sigma";
    tmp_marker2.id = 1;
    tmp_marker2.type = shape;
    // Set the marker action.  Options are ADD and DELETE
    tmp_marker2.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    tmp_marker2.pose.position.x = point.x;
    tmp_marker2.pose.position.y = point.y;
    tmp_marker2.pose.position.z = 0;
    tmp_marker2.pose.orientation.x = 0.0;
    tmp_marker2.pose.orientation.y = 0.0;
    tmp_marker2.pose.orientation.z = 0.0;
    tmp_marker2.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    tmp_marker2.scale.x = .1;
    tmp_marker2.scale.y = .5;
    tmp_marker2.scale.z = 80;

    // Set the color -- be sure to set alpha to something non-zero!
    tmp_marker2.color.r = 0.0f;
    tmp_marker2.color.g = 0.0f;
    tmp_marker2.color.b = 1.0f;
    tmp_marker2.color.a = 1.0f;

    tmp_marker2.lifetime = ros::Duration(0); // How long the object should last before being automatically deleted.  0 means forever
    tmp_marker2.header.stamp = ros::Time::now();

    // Publish the marker
    marker_pub.publish(tmp_marker1);
    marker_pub2.publish(tmp_marker2);

    marker1 = tmp_marker1;
    marker2 = tmp_marker2;
}

tf::Stamped<tf::Pose> LayoutManager::toGlobalFrame(Vector3d p_state)
{
    // Get particle state
    geometry_msgs::PoseStamped pose_local_map_frame;
    pose_local_map_frame.header.frame_id = "local_map";
    pose_local_map_frame.header.stamp = ros::Time::now();
    pose_local_map_frame.pose.position.x = p_state(0);
    pose_local_map_frame.pose.position.y =  p_state(1);
    pose_local_map_frame.pose.position.z =  p_state(2);

    tf::Stamped<tf::Pose> tf_pose_map_frame, tf_pose_local_map_frame;
    tf::poseStampedMsgToTF(pose_local_map_frame, tf_pose_local_map_frame);

    tf_pose_map_frame.setOrigin(tf::Vector3(0,0,0));
    tf_pose_map_frame.setRotation(tf::createIdentityQuaternion());

    // Transform pose from "local_map" to "map"
    try{
         tf_listener.transformPose("map", ros::Time(0), tf_pose_local_map_frame, "local_map", tf_pose_map_frame);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ROS_INFO_STREAM("if(!LayoutManager::first_run){ if(config.particles_number > num_particles)");
        ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
        return tf_pose_map_frame;
    }

    return tf_pose_map_frame;
}


void LayoutManager::reconfigureCallback(road_layout_estimation::road_layout_estimationConfig &config, uint32_t level)
{
    ROS_INFO_STREAM ("Reconfigure callback! " << bagfile );

    // update score guassian distribution values
    street_distribution_sigma     = config.street_distribution_sigma;
    angle_distribution_sigma      = config.angle_distribution_sigma;
    street_distribution_weight    = config.street_distribution_weight;
    angle_distribution_weight     = config.angle_distribution_weight;
    roadState_distribution_weight = config.roadState_distribution_weight;
    resampling_interval           = config.resampling_interval;

    // update uncertainty values -----------------------------------------------------------------------------------
    for(int i=0; i<current_layout.size(); ++i)
    {        

        Particle* particle_ptr = &current_layout.at(i);

        MotionModel* motionModelPointer = particle_ptr->getMotionModelPtr();

        ///disabling
        //motionModelPointer->setErrorCovariance(
        //            config.mtn_model_position_uncertainty,
        //            config.mtn_model_orientation_uncertainty,
        //            config.mtn_model_linear_uncertainty,
        //            config.mtn_model_angular_uncertainty
        //            );

        motionModelPointer->setPropagationError(
                    config.propagate_translational_absolute_vel_error_x,
                    config.propagate_translational_absolute_vel_error_y,
                    config.propagate_translational_absolute_vel_error_z,
                    config.propagate_rotational_absolute_vel_error,
                    config.propagate_translational_percentage_vel_error_x,
                    config.propagate_translational_percentage_vel_error_y,
                    config.propagate_translational_percentage_vel_error_z,
                    config.propagate_rotational_percentage_vel_error
                    );
    }


    // WARNING in teoria questi due non sono più necessari. CHECK! UPDATE: fondamentale per ora è questa parte.
    // motion_model lo abbiamo definito come oggetto della particle.
    // Il Layout manager ha un oggetto default_motion_model che viene copiato in ogni particella.

    // WARNING: this routine is called even by default constructor, may contain OLD values from a older ROSCORE/ROSPARAM
    //          i.e. the values may not be the .cfg/default parameters!

    ///disabling
    //default_mtn_model.setErrorCovariance(
    //            config.mtn_model_position_uncertainty,
    //            config.mtn_model_orientation_uncertainty,
    //            config.mtn_model_linear_uncertainty,
    //            config.mtn_model_angular_uncertainty
    //            );

    default_mtn_model.setPropagationError(
                config.propagate_translational_absolute_vel_error_x,
                config.propagate_translational_absolute_vel_error_y,
                config.propagate_translational_absolute_vel_error_z,
                config.propagate_rotational_absolute_vel_error,
                config.propagate_translational_percentage_vel_error_x,
                config.propagate_translational_percentage_vel_error_y,
                config.propagate_translational_percentage_vel_error_z,
                config.propagate_rotational_percentage_vel_error
                );       

    ///disabling
    // TODO: verificare. perché c'è questa cosa? Non dovrebbe essere letta dal messaggio di odometria? Questa è Q in EKF.
    //measurement_model->setMeasureCov(
    //            config.msr_model_position_uncertainty,
    //            config.msr_model_orientation_uncertainty,
    //            config.msr_model_linear_uncertainty,
    //            config.msr_model_angular_uncertainty
    //            );

    // update particle-set number (only if this IS NOT the first run) ------------------------------------------------
    if(!LayoutManager::layoutManagerFirstRun)
    {
        if(config.particles_number > num_particles)
        {
            // let's add some empty particles to particle-set:
           int counter = current_layout.size(); //this will keep track of current ID
           int particles_to_add = config.particles_number - num_particles;
           for(int i=0; i<particles_to_add; i++)
           {
               ROS_WARN_STREAM("NEW PARTICLE");
               Particle new_particle(counter, default_mtn_model);

               new_particle.in_cluster = -1;
               new_particle.distance_to_closest_segment = 0.0f;

               State6DOF tmp;
               tmp.addNoise(0.5, 0.5, 0.5, 0.5);

               new_particle.setParticleState(tmp);

               // update particle score using OpenStreetMap
               if(LayoutManager::openstreetmap_enabled)
               {
                   tf::Stamped<tf::Pose> tf_pose_map_frame=toGlobalFrame(new_particle.getParticleState().getPose());

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
                       new_particle.setParticleScore(pdf(normal_dist, distance));
                   }
                   else
                   {
                       // Either service is down or particle is too far from a street
                       new_particle.setParticleScore(0);
                   }
               }

               // Push particle
               current_layout.push_back(new_particle);

               // Update counter
               counter = counter+1;
           }// end cycle for creating particle (FOR)
        }
        else if(config.particles_number < num_particles)
        {
            // let's erase particles starting from particle-set tail
            // WARNING: should we remove RANDOM particles instead of 'the last N particles'?
           int particles_to_remove = num_particles - config.particles_number;
           for(int i=0; i<particles_to_remove; i++){
               // delete last element
               current_layout.erase(current_layout.end());
           }
        }

        num_particles = config.particles_number;
    }

    // -----FIRST RUN-------------------------------------------------------------------------------------------
    if(LayoutManager::layoutManagerFirstRun)
    {
        //ROS_ASSERT(num_particles==0);

        if(LayoutManager::openstreetmap_enabled)
        {
            ROS_INFO_STREAM("Road layout manager first run, init particle-set from GPS signal");

            // INITIALIZATION
            double alt  = 0.0f;
            double lat  = 0.0f;
            double lon  = 0.0f;
            double cov1 = 0.0f;
            double cov2 = 0.0f;

            // select RLE mode, hard-coded KITTI initializations, or GPS message
            if (start_with_gps_message)
            {
                // -------- WAIT FOR GPS MESSAGE ----------------------------------------- //

                // wait for GPS message (coming from Android device)
                sensor_msgs::NavSatFix::ConstPtr gps_msg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/kitti_player/oxts/gps");

                // Save values into local vars (this is a trick when GPS device isn't available or we want to simulate a GPS msg)
                alt  = 0.0;
                lon  = gps_msg->longitude;
                lat  = gps_msg->latitude;
                cov1 = gps_msg->position_covariance[0];
                cov2 = gps_msg->position_covariance[4];

                ROS_DEBUG_STREAM("Received GPS lat/lon coordinates: " << lat << "\t" << lon);
                ROS_DEBUG_STREAM("GPS variance in lat/lon " << cov1 << "\t" << cov2);

            }
            else
            {

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


                // KITTI 00 [OK, si impianta dopo un pò per i ritardi accumulati]
                if (bagfile.compare("kitti_00")==0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_00 intial coordinates");
                    alt = 0;
                    lat = 48.9825523586602;//48.98254523586602;
                    lon = 8.39036610004500; //8.39036610004500;
                    cov1 = 15;
                    cov2 = 15;
                }

                // KITTI 01 [OK, video autostrada, si perde nella curva finale]
                if (bagfile.compare("kitti_01")==0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_01 intial coordinates");
                    alt = 0;
                    lat = 49.006719195871;//49.006558;// 49.006616;//
                    lon = 8.4893558806503;//8.489195;//8.489291;//
                    cov1 = 50;
                    cov2 = 50;
                }

                // KITTI 02 [NI, si perde dopo un paio di curve]
                if (bagfile.compare("kitti_02")==0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_02 intial coordinates");
                    alt = 0;
                    lat = 48.987607723096;
                    lon = 8.4697469732634;
                    cov1 = 60;
                    cov2 = 60;
                }

                // KITTI 04 [OK, video road, rettilineo corto]
                if (bagfile.compare("kitti_04")==0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_04 intial coordinates");
                    alt = 0;
                    lat = 49.033603440345;
                    lon = 8.3950031909457;
                    cov1 = 50;
                    cov2 = 50;
                }

                // KITTI 05 [NI, se imbocca la strada giusta nell'inizializzazione funziona bene]
                if (bagfile.compare("kitti_05")==0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_05 intial coordinates");
                    lat = 49.04951961077;
                    lon = 8.3965961639946;
                    alt = 0;
                    cov1 = 4;
                    cov2 = 4;
                }

                // KITTI 06 [OK, video loop, si perde dopo il secondo incrocio]
                if (bagfile.compare("kitti_06")==0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_06 intial coordinates");
                    alt = 0;
                    lat = 49.05349304789598;
                    lon = 8.39721998765449;
                    cov1 = 50;
                    cov2 = 50;
                }

                // KITTI 07 [CUTTED OK, video in cui sta fermo allo stop alcuni secondi]
                if (bagfile.compare("kitti_07")==0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_07 intial coordinates");
                    alt = 0;
                    lat = 48.985319;//48.98523696217;
                    lon = 8.393801;//8.3936414564418;
                    cov1 = 50;
                    cov2 = 50;
                }

                // KITTI 08 [bag inizia dopo]
                if (bagfile.compare("kitti_08")==0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_08 intial coordinates");
                    alt = 0;
                    lat = 48.984311;
                    lon = 8.397817;
                    cov1 = 60;
                    cov2 = 60;
                }

                // KITTI 09 [OK, video serie curve tondeggianti]
                if (bagfile.compare("kitti_09")==0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_09 intial coordinates");
                    alt = 0;
                    lat = 48.972104544468;
                    lon = 8.4761469953335;
                    cov1 = 60;
                    cov2 = 60;
                }

                // KITTI 10 [CUTTED OK, non esegue l'inversione finale rimane indietro]
                if (bagfile.compare("kitti_10")==0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_10 intial coordinates");
                    alt = 0;
                    lat = 48.972406;//48.972455;//48.97253396005;
                    lon = 8.478662;//8.478660;//8.4785980847297;
                    cov1 = 50;
                    cov2 = 50;
                }


            }

            //ros::Duration(1).sleep(); // sleep for 2 secs
                                        // (simulate gps time fix, this will give time to publish poses to Rviz, not needed for RViz 2d pose estimate)

            // Publish GPS fix on map
//            fix.header.frame_id = "map";
//            fix.header.stamp = ros::Time::now();
//            fix.pose.position.x = latlon_2_xy_srv.response.x;
//            fix.pose.position.y = latlon_2_xy_srv.response.y;
//            fix.pose.orientation.w = 1;
//            LayoutManager::gps_pub.publish(fix);


            //// ------------------------ WAIT FOR RVIZ INITIAL POSE  --------------------- //
            //cout << "   Click on '2D pose estimation' in RViz for initialize particle set" << endl;
            //geometry_msgs::PoseWithCovarianceStamped::ConstPtr rviz_msg = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/initialpose");
            //
            //// Get PoseStamped from Rviz msg
            //geometry_msgs::PoseStamped pose_local_map_frame;
            //pose_local_map_frame.pose = rviz_msg->pose.pose;
            //pose_local_map_frame.header = rviz_msg->header;
            //pose_local_map_frame.header.stamp = ros::Time::now(); // Rviz msg has no stamp
            //
            //// Create tf::Pose
            //tf::Stamped<tf::Pose> tf_pose_map_frame, tf_pose_local_map_frame;
            //tf::poseStampedMsgToTF(pose_local_map_frame, tf_pose_local_map_frame);
            //
            //// Transform pose from "map" to "local_map"
            //tf_listener.transformPose("map", ros::Time(0), tf_pose_local_map_frame, "local_map", tf_pose_map_frame);
            //
            //// convert UTM to LatLon
            //ira_open_street_map::xy_2_latlon xy_2_latlon_srv;
            //xy_2_latlon_srv.request.x = tf_pose_map_frame.getOrigin().getX();
            //xy_2_latlon_srv.request.y = tf_pose_map_frame.getOrigin().getY();
            //if (!LayoutManager::xy_2_latlon_client.call(xy_2_latlon_srv)){
            //    ROS_ERROR_STREAM("   Failed to call 'xy_2_latlon' service");
            //    ros::shutdown(); //augusto debug
            //    return;
            //}
            //
            //double alt = 0;
            //double lat = xy_2_latlon_srv.response.latitude;
            //double lon = xy_2_latlon_srv.response.longitude;
            //double cov1 = 150;
            //double cov2 = 150;
            //// ------------------------ END RVIZ INITIAL POSE  ------------------------- //

            // Get XY values from GPS coords
            ira_open_street_map::latlon_2_xy latlon_2_xy_srv;
            latlon_2_xy_srv.request.latitude = lat;
            latlon_2_xy_srv.request.longitude = lon;

            ROS_INFO_STREAM("lat: " << lat << "\t" << "lon: " << lon);

            /// Convert lat/lon to xy
            geometry_msgs::Point point;
            if (LayoutManager::latlon_2_xy_client.call(latlon_2_xy_srv))
            {
                 point.x = latlon_2_xy_srv.response.x;
                 point.y = latlon_2_xy_srv.response.y;
            }
            else
            {
              ROS_ERROR_STREAM("  Error with 'latlon_2_xy_srv' service");
              ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
              return;
            }

            /// Set mean for sampling
            Eigen::Vector2d mean;
            mean.setZero();
            mean << point.x, point.y;

            /// Set covariance for sampling
            Eigen::Matrix2d covar = Eigen::Matrix2d::Identity();
            covar(0,0) = cov1;
            covar(1,1) = cov2;

            ROS_INFO_STREAM ("------------------------------------------------------------" );
            ROS_INFO_STREAM ("GPS FIX COORDINATES:" );
            ROS_INFO_STREAM (std::setprecision(16) << "   lat: " << lat << " lon: " << lon << " alt: " << alt );
            ROS_INFO_STREAM (std::setprecision(16) <<  "   x: " << boost::lexical_cast<std::string>(point.x) << " y: " << boost::lexical_cast<std::string>(point.y) );
            ROS_INFO_STREAM ("MULTIVARIATE PARAMS: " );
            ROS_INFO_STREAM ("   MEAN: " );
            ROS_INFO_STREAM ("       " << boost::lexical_cast<std::string>(mean(0)) );
            ROS_INFO_STREAM ("       " << boost::lexical_cast<std::string>(mean(1)) );
            ROS_INFO_STREAM ("   COV: " );
            ROS_INFO_STREAM (endl << covar );
            ROS_INFO_STREAM ("------------------------------------------------------------" );


            // Publish initial markers (uncertainties areas as a green flat sphere.
            publish_initial_markers(cov1, cov2, point);

            particle_poses_statistics = MatrixXd(config.particles_number,2);

            // Create a bivariate gaussian distribution of doubles.
            // with our chosen mean and covariance
            Eigen::EigenMultivariateNormal<double, 2> normalSampler(mean,covar);

            // Reset current_layout
            LayoutManager::current_layout.clear();

            // Populate current_layout with *ONLY* valid particles
            int particle_id = 1;
            int while_ctr = 1; // while loop infinite cycle prevenction
            while((while_ctr < 1000) && (current_layout.size() < config.particles_number))
            {
                if(while_ctr == 999)
                {
                    ROS_ERROR_STREAM("Random particle set init: while ctr reached max limit" );
                    ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
                    /**
                     * TODO: max_radius_size * 2 and find again
                     */
                }

                // Generate a sample from the bivariate Gaussian distribution
                Matrix<double,2,-1> sample = normalSampler.samples(1);

                //-------------- SNAP PARTICLE v2  -------------------------------------- //
                // Init OSM cartography service
                ira_open_street_map::snap_particle_xy snapParticleXYService;
                snapParticleXYService.request.x = sample(0);
                snapParticleXYService.request.y = sample(1);
                snapParticleXYService.request.max_distance_radius = 200; // distance radius for finding the closest nodes for particle snap
                boost::math::normal street_normal_dist(0, street_distribution_sigma);
                boost::math::normal angle_normal_dist(0, angle_distribution_sigma);

                // Call snap particle service
                if (LayoutManager::snap_particle_xy_client.call(snapParticleXYService))
                {
                    geometry_msgs::PoseStamped pose_map_frame;
                    pose_map_frame.header.frame_id = "map";
                    pose_map_frame.header.stamp = ros::Time::now();
                    pose_map_frame.pose.position.x = snapParticleXYService.response.snapped_x;
                    pose_map_frame.pose.position.y = snapParticleXYService.response.snapped_y;
                    pose_map_frame.pose.position.z = 0.0;

                    snapParticleXYService.response.way_id;

                    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(snapParticleXYService.response.way_dir_rad), pose_map_frame.pose.orientation);
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
                        ROS_ERROR_STREAM("%s"<<ex.what());
                        ROS_INFO_STREAM("map to local map exception");
                        continue; //Skip this iteration
                    }

                    // Init particle's pose
                    State6DOF p_pose;
                    p_pose.setPose(Vector3d(tf_pose_local_map_frame.getOrigin().getX(), tf_pose_local_map_frame.getOrigin().getY(), 0));
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
                    MatrixXd p_sigma = default_mtn_model.getErrorCovariance();

                    ROS_DEBUG_STREAM("p_sigma" << endl << endl << p_sigma << endl);

                    /// Create particle and set its score
                    Particle new_particle(particle_id, p_pose, p_sigma, default_mtn_model);
                    new_particle.setParticleScore(pdf(street_normal_dist,0) * pdf(angle_normal_dist, 0)); // dont' calculate score with distance because particle is snapped
                    // WARNING: in the line above, comment says something different from what is written. furthermore, pdf are not weighted like:
                    // street_distribution_weight * pose_diff_score_component * angle_distribution_weight * final_angle_diff_score
                    ROS_DEBUG_STREAM("Initialized particle with score: " << new_particle.getParticleScore());


                    //////////// CREATE COMPONENT ROAD_STATE ////////////
                    ira_open_street_map::getHighwayInfo getHighwayService;
                    getHighwayService.request.way_id = snapParticleXYService.response.way_id;
                    if(LayoutManager::getHighwayInfo_client.call(getHighwayService))
                    {
                        road_layout_estimation::msg_lines lines;
                        road_layout_estimation::msg_lineInfo lineInfo;

                        lineInfo.isValid = false;
                        lineInfo.offset  = 0.0f;
                        lineInfo.counter = 0;

                        lines.way_id          = snapParticleXYService.response.way_id;
                        lines.width           = getHighwayService.response.width;
                        lines.goodLines       = 0;
                        lines.number_of_lines = Utils::linesFromLanes(getHighwayService.response.number_of_lanes);
                        for (int i = 0; i< getHighwayService.response.number_of_lanes; i++)
                            lines.lines.push_back(lineInfo);


                        ROS_INFO_STREAM("Adding roadState component!");                        
                        LayoutComponent_RoadState *roadState = new LayoutComponent_RoadState(particle_id,
                                                                                             1,                             //component Id
                                                                                             ros::Time::now(),
                                                                                             &this->getHighwayInfo_client,
                                                                                             lines
                                                                                             );

                        //LayoutComponent_RoadState *roadState = new LayoutComponent_RoadState(particle_id,
                        //                                                                     1,
                        //                                                                     snapParticleXYService.response.way_id,
                        //                                                                     getHighwayService.response.number_of_lanes,
                        //                                                                     -1,
                        //                                                                     getHighwayService.response.width,
                        //                                                                     ros::Time::now(),
                        //                                                                     &this->getHighwayInfo_client
                        //                                                                     );

                        VectorXd state=new_particle.getParticleState().getPose();
                        roadState->setComponentState(state);
                        ROS_INFO_STREAM("road state just created: " << roadState->getComponentId() << "\t" << roadState->getComponentState()(0));
                        new_particle.addComponent(roadState);
                    }
                    //////////// CREATE COMPONENT ROAD_STATE ////////////

                    /// Push particle into particle-set
                    current_layout.push_back(new_particle);


                    // Update particles id counter
                    particle_id += 1;

                    // Check if we should create 2 particles with opposite direction
                    if(snapParticleXYService.response.way_dir_opposite_particles)
                    {
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
                        Particle new_particle_opposite(particle_id, p_pose, p_sigma, default_mtn_model);
                        new_particle_opposite.setParticleScore(pdf(street_normal_dist,0) * pdf(angle_normal_dist, 0)); // don't calculate score with distance because particle is snapped
                        // WARNING: in the line above, comment says something different from what is written. furthermore, pdf are not weighted like:
                        // street_distribution_weight * pose_diff_score_component * angle_distribution_weight * final_angle_diff_score
                        ROS_DEBUG_STREAM("Initialized particle with score: " << new_particle.getParticleScore());


                        //////////// CREATE COMPONENT ROAD_STATE ////////////
                        ira_open_street_map::getHighwayInfo getHighwayService;
                        getHighwayService.request.way_id = snapParticleXYService.response.way_id;
                        if(LayoutManager::getHighwayInfo_client.call(getHighwayService))
                        {

                            road_layout_estimation::msg_lines lines;
                            road_layout_estimation::msg_lineInfo lineInfo;

                            lineInfo.isValid = false;
                            lineInfo.offset  = 0.0f;
                            lineInfo.counter = 0;

                            lines.way_id          = snapParticleXYService.response.way_id;
                            lines.width           = getHighwayService.response.width;
                            lines.goodLines       = 0;
                            lines.number_of_lines = Utils::linesFromLanes(getHighwayService.response.number_of_lanes);
                            for (int i = 0; i< getHighwayService.response.number_of_lanes; i++)
                                lines.lines.push_back(lineInfo);

                            ROS_INFO_STREAM("Adding roadState component!");
                            LayoutComponent_RoadState *roadState = new LayoutComponent_RoadState(particle_id,
                                                                                                 1,                             //component Id
                                                                                                 ros::Time::now(),
                                                                                                 &this->getHighwayInfo_client,
                                                                                                 lines
                                                                                                 );

                            //LayoutComponent_RoadState *roadState = new LayoutComponent_RoadState(particle_id,
                            //                                                                     1,
                            //                                                                     snapParticleXYService.response.way_id,
                            //                                                                     getHighwayService.response.number_of_lanes,
                            //                                                                     -1,
                            //                                                                     getHighwayService.response.width,
                            //                                                                     ros::Time::now(),
                            //                                                                     &this->getHighwayInfo_client
                            //                                                                     );

                            VectorXd state=new_particle_opposite.getParticleState().getPose();
                            (*roadState).setComponentState(state);
                            new_particle_opposite.addComponent(roadState);
                        }
                        //////////// CREATE COMPONENT ROAD_STATE ////////////

                        /// Push particle inside particle-set
                        current_layout.push_back(new_particle_opposite);

                        // Update particles id counter
                        particle_id += 1;
                        //we're adding one more particle , one more than expected/requested
                        num_particles++;
                    }
                }
                else
                {
                    ROS_ERROR_STREAM("     Failed to call 'snap_particle_xy' service");
                    ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
                    return;
                }

                // prevent infinite loop
                while_ctr += 1;
            } // end while loop for particles generations
        } // end if(gps_initialization using OSM maps)
        else
        {
            // gps initialization disabled, just generate particles with all values set to zero
            for(int i = 0; i < config.particles_number; i++){
                Particle zero_part(i, default_mtn_model);
                MatrixXd tmp_cov = default_mtn_model.getErrorCovariance();
                zero_part.setParticleSigma(tmp_cov);
                current_layout.push_back(zero_part);
            }
        }

        /// Update particle_set size
        LayoutManager::num_particles = config.particles_number + num_particles; // +num_particles because we can have more particles than expected since the driving direction

        // Update first_run flag
        // LayoutManager::layoutManagerFirstRun = false; //moving this to LayoutEstimation Loop

        // print particle poses
        vector<Particle>::iterator particle_itr;
        ROS_ERROR_STREAM("NUMERO DI LAYOUT: " << current_layout.size());
        for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ )
        {
            ostringstream tmp_convert;   // stream used for the conversion
            tmp_convert << "Particle ID: " << (*particle_itr).getId();
            (*particle_itr).getParticleState().printState(tmp_convert.str());
        }


        // BUILD POSEARRAY MSG
        // Get particle-set
        geometry_msgs::PoseArray array_msg = LayoutManager::buildPoseArrayMsg(current_layout);
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
        ROS_DEBUG_STREAM("PUBLISHING INITIAL ARROWS: " << i);
        Particle p = current_layout.at(i);
        geometry_msgs::Pose pose = p.getParticleState().toGeometryMsgPose();

        tf::Quaternion q;
        tf::quaternionMsgToTF(pose.orientation,q);
        q = q.normalize();
        tf::quaternionTFToMsg(q, pose.orientation);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "local_map";
        marker.header.stamp = ros::Time();
        marker.ns = "particle_set";
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

void LayoutManager::publishMarkerArrayDistances(int id, double x1, double y1, double x2, double y2, double z)
{
    /// Distances between the poses and the closest road segment. For debuggin purposes.
    ///
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

void LayoutManager::odometryCallback(const nav_msgs::Odometry& visualOdometryMsg)
{
    ROS_INFO_STREAM("--------------------------------------------------------------------------------");
    ROS_INFO_STREAM("Entering OdomCallBackv2, [odometryMessageCounter: " << odometryMessageCounter++ << "]");
    vector<Particle>::iterator particle_itr;

    // Publish GPS init markers
    /// useless since the markers does not expire (should not expire)
    // marker_pub.publish(marker1);
    // marker_pub2.publish(marker2);

    if(LayoutManager::first_msg)
    {
        visualOdometryOldMsg = visualOdometryMsg; //used for timestamps only
        // update flag
        LayoutManager::first_msg = false;
        ROS_WARN_STREAM("Setting LayoutManager::first_msg = false");
    }


    // calculate delta_t
    deltaOdomTime = visualOdometryMsg.header.stamp.toSec() - visualOdometryOldMsg.header.stamp.toSec();
    ROS_DEBUG_STREAM("void LayoutManager::odometryCallback2 Odometry Timestamp: " << visualOdometryMsg.header.stamp);
    ROS_DEBUG_STREAM("void LayoutManager::odometryCallback2 Previous Timestamp: " << visualOdometryOldMsg.header.stamp);
    ROS_DEBUG_STREAM("void LayoutManager::odometryCallback2 Delta Timestamp    : "<< deltaOdomTime);

    // retrieve measurement from odometry
    measurement_model->setMsg(visualOdometryMsg);


    // % LINES
    publisher_marker_array_distances.publish(marker_array_distances);

    // % Z
    publisher_z_snapped.publish(marker_z_snapped);
    publisher_z_particle.publish(marker_z_particle);
    // -------------------------------------------------------------------------------------------------------------------------------------

    // -------------------------------------------------------------------------------------------------------------------------------------
    // BUILD POSEARRAY MSG
    // Get particle-set
    /// **disabling arrows
    ///vector<Particle> particles = current_layout;
    ///geometry_msgs::PoseArray array_msg = LayoutManager::buildPoseArrayMsg(particles);
    ///array_msg.header.stamp = ros::Time::now();//msg.header.stamp;

    // Publish it!
    //array_pub.publish(array_msg);
    /// moving to timer loop routine
    //LayoutManager::publishMarkerArray();
    // -------------------------------------------------------------------------------------------------------------------------------------

    // Update old_msg with current one for next step delta_t calculation
    visualOdometryOldMsg = visualOdometryMsg;


    // COLLECTING RESULTS && STATISTICS
    State6DOF state;
    Vector3d average_pose;
    average_pose.setZero();
    //Eigen::Quaterniond average_quaternion = Eigen::Quaterniond::Identity();
    tf::Quaternion average_quaternion = tf::createIdentityQuaternion();
    average_quaternion.setX(0);
    average_quaternion.setY(0);
    average_quaternion.setZ(0);
    average_quaternion.setW(0);

    // CALCULATING TOTAL SCORE FOR NORMALIZATION (used in clustering)
    double tot_score = 0.0d;
    //for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ ){
    ////  ROS_INFO_STREAM("PARTICLE "<< std::setprecision(5) << (*particle_itr).getId() << "\t" << (*particle_itr).getParticleScore() << "\t" << tot_score);
    //    tot_score += (*particle_itr).getParticleScore();
    //}

    ///////////////////////////////////////////////////////////////////////////
    // CLUSTERING PHASE
    bool enabled_clustering = false;
    int best_cluster=-1;
    int best_cluster_size=0;
    double cluster_score=0.0f;
    double best_cluster_score = -1.0f;
    if (enabled_clustering)
    {
        ///////////////////////////////////////////////////////////////////////////
        // FIRST STEP, every particle in_cluster value = -1
        for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ )
        {
            (*particle_itr).in_cluster = -1;
        }
        ///////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////
        // SECOND STEP, clustering. double for.
        int cluster_INDEX=0;
        double euclidean_distance=0.0f; double angle_distance=0.0f;
        double euclidean_threshold = 5.00f; //meters
        double angle_threshold     = 0.20f; //radians
        vector<Particle>::iterator inner_particle_itr;
        for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ ){

            if ((*particle_itr).in_cluster == -1)
                (*particle_itr).in_cluster = cluster_INDEX++;

            for( inner_particle_itr = current_layout.begin(); inner_particle_itr != current_layout.end(); inner_particle_itr++ ){
                if (particle_itr == inner_particle_itr)
                    continue;
                if ((*inner_particle_itr).in_cluster == -1)
                    continue;

                euclidean_distance = sqrt(   (*particle_itr).getParticleState().getPose()(0)-(*inner_particle_itr).getParticleState().getPose()(0)+
                                             (*particle_itr).getParticleState().getPose()(1)-(*inner_particle_itr).getParticleState().getPose()(1)+
                                             (*particle_itr).getParticleState().getPose()(2)-(*inner_particle_itr).getParticleState().getPose()(2)
                                             );

                Eigen::Quaterniond q1 = Eigen::Quaterniond((*particle_itr).getParticleState().getRotation());
                Eigen::Quaterniond q2 = Eigen::Quaterniond((*inner_particle_itr).getParticleState().getRotation());
                tf::Quaternion t1,t2;
                t1.setX(q1.x());t2.setX(q2.x());
                t1.setY(q1.y());t2.setY(q2.y());
                t1.setZ(q1.z());t2.setZ(q2.z());
                t1.setW(q1.w());t2.setW(q2.w());

                angle_distance = t1.angleShortestPath(t2);

                if (euclidean_distance < euclidean_threshold)
                    if (angle_distance < angle_threshold)
                        (*inner_particle_itr).in_cluster  = (*particle_itr).in_cluster;

            }
        }
        ///////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////
        //STEP3 - searching best cluster
        vector<double> clusters;
        clusters.resize(cluster_INDEX);
        for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ ){
            clusters[(*particle_itr).in_cluster]+=(*particle_itr).getParticleScore() / tot_score;
        }

        best_cluster=-1;
        best_cluster_score=-1.0f;
        for(int looking_for_best_cluster = 0; looking_for_best_cluster < cluster_INDEX; looking_for_best_cluster++ )
        {
            if (clusters[looking_for_best_cluster]>best_cluster_score)
            {
                best_cluster_score=clusters[looking_for_best_cluster];
                best_cluster=looking_for_best_cluster;
            }
        }
        cluster_score=0.0f;
        best_cluster_size=0;
        for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ ){
            if ((*particle_itr).in_cluster == best_cluster)
            {
                cluster_score += (*particle_itr).getParticleScore();
                best_cluster_size++;
            }
        }
        cout << "Best cluster size: " << best_cluster_size << endl;
        ///////////////////////////////////////////////////////////////////////////
    }


    ///////////////////////////////////////////////////////////////////////////
    // CREATING STATISTICS FOR RLE OUTPUT

    bool enabled_statistics = false;
    if (enabled_statistics)
    {
        double average_distance=0.0f;
        for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ )
        {
            //cout << "particle in_cluster: " << (*particle_itr).in_cluster << " and the best is: " << best_cluster << endl;

            // CHECK IF THE PARTICLE IS IN THE BEST CLUSTER
            if (enabled_clustering && ((*particle_itr).in_cluster != best_cluster) )
                continue;

            state = (*particle_itr).getParticleState();
            if (enabled_clustering)
                average_pose += state.getPose() * (*particle_itr).getParticleScore() / cluster_score; //tot_score;
            else
                average_pose += state.getPose() * (*particle_itr).getParticleScore() / tot_score;

            //sum += (*particle_itr).getParticleScore() / tot_score;
            Eigen::Quaterniond q = Eigen::Quaterniond(state.getRotation());
            tf::Quaternion t;
            t.setX(q.x());
            t.setY(q.y());
            t.setZ(q.z());
            t.setW(q.w());

            if (enabled_clustering)
                average_quaternion += t.slerp(tf::createIdentityQuaternion(),(*particle_itr).getParticleScore() / cluster_score);
            else
                average_quaternion += t.slerp(tf::createIdentityQuaternion(),(*particle_itr).getParticleScore() / tot_score);

            if (enabled_clustering)
                average_distance += (*particle_itr).distance_to_closest_segment * (*particle_itr).getParticleScore() / cluster_score;
            else
                average_distance += (*particle_itr).distance_to_closest_segment * (*particle_itr).getParticleScore() / tot_score;
        }
        average_quaternion.normalize();
        ///////////////////////////////////////////////////////////////////////////



        nav_msgs::Odometry odometry;
        odometry.header.frame_id="/local_map";
        odometry.header.stamp=ros::Time::now();

        odometry.pose.pose.position.x=average_pose(0);
        odometry.pose.pose.position.y=average_pose(1);
        odometry.pose.pose.position.z=average_pose(2);
        tf::quaternionTFToMsg(average_quaternion,odometry.pose.pose.orientation);
        double roll=0.0f, pitch=0.0f, yaw=0.0f;

        publisher_average_pose.publish(odometry); // Odometry message

        ifstream RTK;
        double from_latitude,from_longitude,from_altitude,to_lat,to_lon;
        //TODO: find an alternative to this shit
        cout << "/media/limongi/Volume/KITTI_RAW_DATASET/BAGS/"+bagfile.substr(bagfile.find_last_of("_")+1,2)+"/oxts/data/" << boost::str(boost::format("%010d") % visualOdometryMsg.header.seq ) <<  ".txt" << endl;
        RTK.open(((string)("/media/limongi/Volume/KITTI_RAW_DATASET/BAGS/"+bagfile.substr(bagfile.find_last_of("_")+1,2)+"/oxts/data/" + boost::str(boost::format("%010d") % visualOdometryMsg.header.seq ) + ".txt")).c_str());
        if (!RTK.is_open())
        {
            cout << "ERROR OPENING THE extraordinary kind FILE!" << endl;
            ros::shutdown();
        }
        RTK >> from_latitude >> from_longitude >> from_altitude;
        cout <<  "LAT LON FROM GPS FILE " << from_latitude << "\t" << from_longitude << endl;
        RTK.close();

        sensor_msgs::NavSatFix gps_fix;
        gps_fix.header.frame_id="/map";
        gps_fix.header.stamp = visualOdometryMsg.header.stamp;
        gps_fix.latitude=from_latitude;
        gps_fix.longitude=from_longitude;
        gps_fix.altitude=from_altitude;
        gps_pub.publish(gps_fix);


        /*
     *      SAVING GPS-RTK PART
     */
        // Get XY values from GPS coords
        ira_open_street_map::latlon_2_xyRequest query_latlon2xy;
        query_latlon2xy.latitude = from_latitude;
        query_latlon2xy.longitude = from_longitude;
        ira_open_street_map::latlon_2_xyResponse response_latlon2xy;
        if (LayoutManager::latlon_2_xy_client.call(query_latlon2xy,response_latlon2xy))
        {
            cout << std::setprecision(16) << response_latlon2xy;
            tf::Stamped<tf::Pose> RTK_map_frame, RTK_local_map_frame;
            RTK_map_frame.setOrigin(tf::Vector3(response_latlon2xy.x,response_latlon2xy.y,0));
            RTK_map_frame.setRotation(tf::createIdentityQuaternion());
            RTK_map_frame.frame_id_="/map";

            // Transform pose from "map" to "local_map"
            try{
                tf_listener.transformPose("local_map", ros::Time(0), RTK_map_frame, "map", RTK_local_map_frame);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR_STREAM("LayoutManager.cpp says: %s"<<ex.what());
                ROS_ERROR_STREAM("     Transform RTK pose from map to local_map");
                ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
            }

            visualization_msgs::Marker RTK_MARKER;
            RTK_MARKER.header.frame_id = "local_map";
            RTK_MARKER.header.stamp = ros::Time();
            RTK_MARKER.ns = "RTK_MARKER";
            RTK_MARKER.id = visualOdometryMsg.header.seq; // same as image from kitti dataset
            RTK_MARKER.type = visualization_msgs::Marker::CYLINDER;
            RTK_MARKER.action = visualization_msgs::Marker::ADD;
            RTK_MARKER.pose.orientation.w = 1;
            RTK_MARKER.scale.x = 0.5;
            RTK_MARKER.scale.y = 0.5;
            RTK_MARKER.scale.z = 0.5;
            RTK_MARKER.color.a = 1.0;
            RTK_MARKER.color.r = 0;
            RTK_MARKER.color.g = 1.0;
            RTK_MARKER.color.b = 0.0;
            RTK_MARKER.pose.position.x = RTK_local_map_frame.getOrigin().getX();//response.x;
            RTK_MARKER.pose.position.y = RTK_local_map_frame.getOrigin().getY();//response.y;
            RTK_MARKER.pose.position.z = RTK_local_map_frame.getOrigin().getZ();//;

            marker_array_GT_RTK.markers.push_back(RTK_MARKER);

            // Push back line_list
            publisher_GT_RTK.publish(marker_array_GT_RTK);

            RTK_GPS_out_file << visualOdometryMsg.header.seq << " " << setprecision(16) <<
                                RTK_local_map_frame.getOrigin().getX() << " " << RTK_local_map_frame.getOrigin().getY() << " " << RTK_local_map_frame.getOrigin().getZ() << " " <<
                                0 << " "<< 0 << " "<< 0 << " " <<
                                0 << " " << 0 << " " << 0 << " " << 0 << " " <<
                                tot_score / current_layout.size() << " " <<
                                query_latlon2xy.latitude << " " << query_latlon2xy.longitude << "\n";


            //        cout  << msg.header.seq << " " << setprecision(16) <<
            //                            RTK_local_map_frame.getOrigin().getX() << " " << RTK_local_map_frame.getOrigin().getY() << " " << RTK_local_map_frame.getOrigin().getZ() << " " <<
            //                            0 << " "<< 0 << " "<< 0 << " " <<
            //                            0 << " " << 0 << " " << 0 << " " << 0 << " " <<
            //                            tot_score / current_layout.size() << " " <<
            //                            query_latlon2xy.latitude << " " << query_latlon2xy.longitude << "\n";
        }
        else
        {
            ROS_ERROR_STREAM("   Failed to call 'latlon_2_xy_srv' service");
            ros::shutdown(); //augusto debug
            return;
        }



        /*
     *      SAVING RLE PART
     */
        // TRANSFORM AVERAGE POSE TO LAT/LON (NEED CONVERSION FROM LOCAL_MAP TO MAP AND ROS-SERVICE CALL)
        tf::Stamped<tf::Pose> average_pose_map_frame, average_pose_local_map_frame;
        average_pose_local_map_frame.frame_id_="local_map";
        average_pose_local_map_frame.setOrigin(tf::Vector3(average_pose(0),average_pose(1),average_pose(2)));
        average_pose_local_map_frame.setRotation(tf::createIdentityQuaternion());
        // Transform pose from "local_map" to "map"
        try{
            tf_listener.transformPose("map", ros::Time(0), average_pose_local_map_frame, "local_map", average_pose_map_frame);

            ira_open_street_map::xy_2_latlonRequest query_xy2latlon;
            ira_open_street_map::xy_2_latlonResponse response_xy2latlon;
            query_xy2latlon.x=average_pose_map_frame.getOrigin().getX();
            query_xy2latlon.y=average_pose_map_frame.getOrigin().getY();
            if (LayoutManager::xy_2_latlon_client.call(query_xy2latlon,response_xy2latlon))
            {
                // to_lat && to_lon are then the average values (of all particles) in LAT/LON UTM
                to_lat=response_xy2latlon.latitude;
                to_lon=response_xy2latlon.longitude;
            }
            else
            {
                ROS_ERROR_STREAM("   Failed to call 'xy_2_latlon_2_srv' service");
                ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
                return;
            }

            // -------------------------------------------------------------------------------------------------------------------------------------
            // SAVE RESULTS TO OUTPUT FILE:
            tf::Matrix3x3(average_quaternion).getRPY(roll,pitch,yaw);
            RLE_out_file << visualOdometryMsg.header.seq << " " << setprecision(16) <<
                            average_pose(0) << " " << average_pose(1) << " " << average_pose(2) << " " <<
                            roll << " "<< pitch << " "<< yaw << " " <<
                            average_quaternion.getX() << " " << average_quaternion.getY() << " " << average_quaternion.getZ() << " " << average_quaternion.getW() << " " <<
                            tot_score / current_layout.size() << " " <<
                            to_lat << " " << to_lon << " " <<
                            average_distance << "\n";

            cout << visualOdometryMsg.header.seq << " " << setprecision(16) <<
                    average_pose(0) << " " << average_pose(1) << " " << average_pose(2) << " " <<
                    roll << " "<< pitch << " "<< yaw << " " <<
                    average_quaternion.getX() << " " << average_quaternion.getY() << " " << average_quaternion.getZ() << " " << average_quaternion.getW() << " " <<
                    tot_score / current_layout.size() << " " <<
                    to_lat << " " << to_lon << " " <<
                    average_distance << "\n";
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR_STREAM("LayoutManager.cpp says: %s"<<ex.what());
            ROS_ERROR_STREAM("     Transform AVERAGE pose from local_map to map");
            ros::shutdown();
        }


        /*
     *      SAVING LIBVISO PART
     */

        // TODO: calculate LAT LON
        tf::StampedTransform VO;
        try{
            tf_listener.lookupTransform("/local_map","/visual_odometry_car_frame",ros::Time(0),VO);

            tf::Matrix3x3(VO.getRotation()).getRPY(roll,pitch,yaw);

            LIBVISO_out_file << visualOdometryMsg.header.seq << " " << setprecision(16) <<
                                VO.getOrigin().getX()  << " " << VO.getOrigin().getY()  << " " << VO.getOrigin().getZ()  << " " <<
                                roll << " "<< pitch << " "<< yaw << " " <<
                                VO.getRotation().getX() << " " << VO.getRotation().getY() << " " << VO.getRotation().getZ() << " " << VO.getRotation().getW() << " " <<
                                tot_score / current_layout.size() << "\n";
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN_STREAM("VO");
        }
    }
    // -------------------------------------------------------------------------------------------------------------------------------------

    ROS_INFO_STREAM("Exiting OdomCallBack v2");

    //Start the timer.  Does nothing if the timer is already started.
    this->rleStart();
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
void LayoutManager::roadLaneCallback(const road_lane_detection::road_lane_array& msg)
{
    ROS_ERROR_STREAM("> Entering roadLaneCallback");

    ROS_ERROR_STREAM("WHY ARE YOU CALLING THIS ROUTINE?!");

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

    ROS_ERROR_STREAM("< Exiting roadLaneCallback");
}

void LayoutManager::roadStateCallback(const road_layout_estimation::msg_lines &msg_lines)
{

    ROS_INFO_STREAM("> Entering roadStateCallback");

    ira_open_street_map::snap_particle_xy snapParticle;

    road_layout_estimation::msg_lines modified_msg_lines;
    modified_msg_lines=msg_lines;


    for(int i=0; i<current_layout.size(); ++i)
    {
        // Get all layout components of particle
        Particle* particle = &current_layout.at(i);
        vector<LayoutComponent*>* layout_components = particle->getLayoutComponentsPtr();

        /// Need the way_id for component evaluation
        /// Since it is not "detected" retrieve it from the particle state.
        /// TODO: handle this, maybe we can add this in the propagateComponent etc

        tf::Stamped<tf::Pose> tf_pose_map_frame=toGlobalFrame(particle->getParticleState().getPose());
        snapParticle.request.x = tf_pose_map_frame.getOrigin().getX();
        snapParticle.request.y = tf_pose_map_frame.getOrigin().getY();
        snapParticle.request.max_distance_radius = 20;  // TODO: parametrize this value

        if (snap_particle_xy_client.call(snapParticle))
        {
            ROS_DEBUG_STREAM("Snap OK in roadStateCallback " << snapParticle.response.way_id;);
            modified_msg_lines.way_id = snapParticle.response.way_id;
        }
        else
        {
            ROS_WARN_STREAM("Warning, snap failed in roadStateCallback in the radius of " << snapParticle.request.max_distance_radius << "m");
            snapParticle.response.way_id=0;
        }

        // Clear old layout_components
        layout_components->clear(); //this should be replaced with some more clever idea... (maybe update the component instead of deleting them)

        ROS_DEBUG_STREAM("Adding roadState component! way_id: " << modified_msg_lines.way_id << "\tDetected lanes: " << modified_msg_lines.number_of_lines << "\tDetected width: " << modified_msg_lines.width);

        //LayoutComponent_RoadState *roadState = new LayoutComponent_RoadState(particle->getId(),
        //                                                                     1,                             //component Id
        //                                                                     msg_lanes.way_id,
        //                                                                     msg_lanes.number_of_lanes,
        //                                                                     -1,                            //curren lane
        //                                                                     msg_lanes.width,
        //                                                                     ros::Time::now(),
        //                                                                     &this->getHighwayInfo_client
        //                                                                     );


        // I need to get the way_id value of the particle and put it to the message in order to enable the computevalue of the component.
        // this was included in the fake detector before.

        LayoutComponent_RoadState *roadState = new LayoutComponent_RoadState(particle->getId(),
                                                                             1,                             //component Id
                                                                             ros::Time::now(),
                                                                             &this->getHighwayInfo_client,
                                                                             modified_msg_lines
                                                                             );

        VectorXd state=particle->getParticleState().getPose();
        roadState->setComponentState(state);
        ROS_DEBUG_STREAM("roadState just created, ComponentID: " << roadState->getComponentId() << "\tState: " << roadState->getComponentState()(0) << " " << roadState->getComponentState()(1) << " " << roadState->getComponentState()(2)<< "\tWay_id: "<< roadState->getWay_id() );

        // Add lane to layout components
        particle->addComponent(roadState);
    }

    ROS_INFO_STREAM("< Exiting roadStateCallback");
}



/** **************************************************************************************************************/
/**
 * Estimate particles' components using particle filter
 */
void LayoutManager::componentsEstimation()
{
    ROS_DEBUG("> Entering componentsEstimation");

    /// STEP 1: SAMPLING (PREDICT COMPONENTS POSES)
    sampling();

    //// STEP 2: PERTURBATE COMPONENT POSES
    componentsPerturbation();

    //// STEP 3: WEIGHT LAYOUT-COMPONENTS
    calculateLayoutComponentsWeight();

    ROS_DEBUG("< Exiting componentsEstimation");
}

/**
 * PARTICLE FILTER, STEP 1:
 * cicle through all the particles, and call their function "propagate-components"
 */
void LayoutManager::sampling(){
    ROS_DEBUG_STREAM("> Entering Sampling of all components");
    vector<Particle>::iterator itr;
    for( itr = current_layout.begin(); itr != current_layout.end(); itr++ ){
        ROS_INFO_STREAM_ONCE("--- Propagating components of particle, ID: " << itr->getId() << " ---");

        // QtCreator annoying bug... solved with this 'trick'
        Particle &particle=*itr;
        particle.propagateLayoutComponents();

        //itr->propagateLayoutComponents();
    }
    ROS_DEBUG_STREAM("< Exiting Sampling of all components");
}

/**
 * PARTICLE FILTER, STEP 2:
 * @brief LayoutManager::componentsPerturbation
 */
void LayoutManager::componentsPerturbation()
{

    // first, iterate over all particles of 'current_layout'
    for(int i=0; i<current_layout.size(); i++){
        Particle p = current_layout.at(i);
        vector<LayoutComponent*> vec = p.getLayoutComponents();

        // second, iterate over all layout-components of current 'particle'
        for(int j=0; j<vec.size(); j++)
        {
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
void LayoutManager::calculateLayoutComponentsWeight()
{
    ROS_DEBUG_STREAM("> Entering calculateLayoutComponentsWeight (and then calling *virtual* calculateComponentScore");
    // first, iterate over all particles of 'current_layout'
    for(int i=0; i<current_layout.size(); i++)
    {
        Particle p = current_layout.at(i);
        ROS_DEBUG_STREAM("Particle p.getId() : " << p.getId());
        vector<LayoutComponent*> vec = p.getLayoutComponents();

        // second, iterate over all layout-components of current 'particle'
        for(int j=0; j<vec.size(); j++)
        {
            LayoutComponent* lc = vec.at(j);
            lc->calculateComponentScore();
        }
    }
    ROS_DEBUG_STREAM("< Exiting calculateLayoutComponentsWeight (and then calling *virtual* calculateComponentScore");
}
/** **************************************************************************************************************/

void LayoutManager::calculateGeometricScores(Particle *particle_itr)
{
    // SCORE using OpenStreetMap-----------------------------------------------------------------------------
    // update particle score using OpenStreetMap
    if(LayoutManager::openstreetmap_enabled)
    {
        // Diff quaternions used for angle score
        tf::Quaternion first_quaternion_diff;
        tf::Quaternion second_quaternion_diff;
        tf::Quaternion street_direction;

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
            ROS_ERROR_STREAM("%s"<<ex.what());
            ROS_ERROR_STREAM("     Transform snapped particle pose from local_map to map");
            ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
            return;
        }

        ///Augusto: TEST 0 - Send TF Transforms (MSG & TF) local_map > pose; Them shuld be the same
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
        ira_open_street_map::snap_particle_xy snapParticle_serviceMessage;
        snapParticle_serviceMessage.request.x = tf_pose_map_frame.getOrigin().getX();
        snapParticle_serviceMessage.request.y = tf_pose_map_frame.getOrigin().getY();
        snapParticle_serviceMessage.request.max_distance_radius = 100; // distance radius for finding the closest nodes for particle snap

        ira_open_street_map::getDistanceFromLaneCenter getDistanceFromLaneCenter_serviceMessage;
        getDistanceFromLaneCenter_serviceMessage.request.x = tf_pose_map_frame.getOrigin().getX();
        getDistanceFromLaneCenter_serviceMessage.request.y = tf_pose_map_frame.getOrigin().getY();
        getDistanceFromLaneCenter_serviceMessage.request.way_id = (dynamic_cast<LayoutComponent_RoadState *>((*particle_itr).getLayoutComponents().at(0)))->getWay_id(); //FIXME: find the right component (check also the cast)
        getDistanceFromLaneCenter_serviceMessage.request.current_lane = 0; //FIX: unused?!


        if (LayoutManager::getDistanceFromLaneCenter_client.call(getDistanceFromLaneCenter_serviceMessage))
        {
            //ROS_ERROR_STREAM ("STICA!   " << getDistanceFromLaneCenter_serviceMessage.request.way_id << "\t " << getDistanceFromLaneCenter_serviceMessage.response.distance_from_lane_center << "\t" << getDistanceFromLaneCenter_serviceMessage.response.distance_from_way_center);

        }

        // Get distance from snapped particle pose and set it as particle score
        if (LayoutManager::snap_particle_xy_client.call(snapParticle_serviceMessage))
        {
            // Snapped pose is is map frame, convert from MSG to TF first.
            geometry_msgs::PoseStamped snapped_map_frame;
            snapped_map_frame.header.frame_id = "map";
            snapped_map_frame.header.stamp = ros::Time::now();
            snapped_map_frame.pose.position.x = snapParticle_serviceMessage.response.snapped_x;
            snapped_map_frame.pose.position.y =  snapParticle_serviceMessage.response.snapped_y;
            snapped_map_frame.pose.position.z =  0;
            snapped_map_frame.pose.orientation.x = snapParticle_serviceMessage.response.way_dir_quat_x;
            snapped_map_frame.pose.orientation.y = snapParticle_serviceMessage.response.way_dir_quat_y;
            snapped_map_frame.pose.orientation.z = snapParticle_serviceMessage.response.way_dir_quat_z;
            snapped_map_frame.pose.orientation.w = snapParticle_serviceMessage.response.way_dir_quat_w;

            tf::Stamped<tf::Pose> tf_snapped_map_frame, tf_snapped_local_map_frame, tf_snapped_local_map_frame_opposite_direction ;
            tf::poseStampedMsgToTF(snapped_map_frame, tf_snapped_map_frame);

            ///Augusto: TEST 1 ---- BE CAREFUL, THIS MAY NOT BE THE RIGHT DIRECTION!
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

            ///Augusto: TEST 2. CHECKED LATER IN THE CODE, BUT KNOW BY ME.
            //if (srv.response.way_dir_opposite_particles)
            //{
            //    tf_snapped_map_frame.setRotation(tf_snapped_map_frame*tf::createQuaternionFromYaw(M_PI));
            //
            //    transform.setOrigin( tf_snapped_map_frame.getOrigin());
            //    transform.setRotation(tf_snapped_map_frame.getRotation());
            //    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "tf_snapped_map_frame_INVERTED_DIRECTION"));
            //
            //    // ** THIS ROUTING LEAVES THE ROTATION INVERTED **
            //}

            // Transform pose from "map" to "local_map"
            try{
                tf_listener.waitForTransform("local_map","map",ros::Time(0),ros::Duration(1));
                tf_listener.transformPose("local_map", ros::Time(0), tf_snapped_map_frame, "map", tf_snapped_local_map_frame);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR_STREAM("RLE MAIN LOOP");
                ROS_ERROR_STREAM("%s" << ex.what());
                ROS_ERROR_STREAM("     Transform snapped particle pose from map to local_map");
                ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
                return;
            }

            ///Augusto: TEST 3. ** BE CAREFUL, THIS IS EQUAL TO INVERTED DIRECTION IF TEST2 IS ENABLED .
            transform.setOrigin( tf_snapped_local_map_frame.getOrigin());
            transform.setRotation(tf_snapped_local_map_frame.getRotation());
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_map", "tf_snapped_local_map_frame"));

            // calculate distance from original particle positin and snapped particle position ---------------------------------
            // use it for score calculation with normal distribution PDF
            double dx = tf_pose_local_map_frame.getOrigin().getX() - tf_snapped_local_map_frame.getOrigin().getX();
            double dy = tf_pose_local_map_frame.getOrigin().getY() - tf_snapped_local_map_frame.getOrigin().getY();
            double dz = tf_pose_local_map_frame.getOrigin().getZ() - 0; // particle Z axis is forced to be next to zero
            double distance = sqrt(dx*dx + dy*dy + dz*dz);

            /// For debuggin purposes: add line to marker array distances
            //publishMarkerArrayDistances((*particle_itr).getId(),
            //                            tf_pose_local_map_frame.getOrigin().getX(),
            //                            tf_pose_local_map_frame.getOrigin().getY(),
            //                            tf_snapped_local_map_frame.getOrigin().getX(),
            //                            tf_snapped_local_map_frame.getOrigin().getY(),
            //                            tf_pose_local_map_frame.getOrigin().getZ());

            //      get PDF score FOR DISTANCE
            boost::math::normal normal_dist(0, street_distribution_sigma);
            //double pose_diff_score_component = pdf(normal_dist, distance);
            (*particle_itr).pose_diff_score_component = pdf(normal_dist, distance) / pdf(normal_dist , 0.0f);
            //tf_snapped_local_map_frame.getRotation().getAngleShortestPath(tf_pose_local_map_frame.getRotation()) // check this out, maybe works .. this line isn't tested yet

            // calculate angle difference ---------------------------------------------------------------------------------------
            first_quaternion_diff = tf_snapped_local_map_frame.getRotation().inverse() * tf_pose_local_map_frame.getRotation();
            street_direction=first_quaternion_diff;

            //      get PDF score from first angle
            boost::math::normal angle_normal_dist(0, angle_distribution_sigma);
            double first_angle_difference = Utils::normalize_angle(first_quaternion_diff.getAngle());
            double first_angle_diff_score = pdf(angle_normal_dist, first_angle_difference) / pdf(angle_normal_dist,0.0f);

            (*particle_itr).final_angle_diff_score_component  = 0.0f;
            double second_angle_diff_score = 0.0f;
            double second_angle_difference = 0.0f;

            //      if street have 2 directions check angle diff with opposite angle
            if(snapParticle_serviceMessage.response.way_dir_opposite_particles)
            {
                tf_snapped_local_map_frame_opposite_direction=tf_snapped_local_map_frame; // COPY TRANSFORM (I PRAY FOR THIS)
                tf_snapped_local_map_frame_opposite_direction.setRotation(tf_snapped_local_map_frame*tf::createQuaternionFromYaw(M_PI)); // INVERT DIRECTION
                second_quaternion_diff = tf_snapped_local_map_frame_opposite_direction.getRotation().inverse() * tf_pose_local_map_frame.getRotation();

                //      get PDF score
                second_angle_difference = Utils::normalize_angle(second_quaternion_diff.getAngle());
                second_angle_diff_score = pdf(angle_normal_dist, second_angle_difference);

                //      set score
                if(second_angle_diff_score > first_angle_diff_score)
                    (*particle_itr).final_angle_diff_score_component = second_angle_diff_score;
                else
                    (*particle_itr).final_angle_diff_score_component = first_angle_diff_score;

                ROS_DEBUG_STREAM("ONEWAY-No \tSPATIAL_DISTANCE: "<< distance << "\tANGLE_1: " << first_angle_difference << "\tANGLE_2: " << second_angle_difference);
            }
            else
            {
                ROS_DEBUG_STREAM("ONEWAY-Yes\tSPATIAL_DISTANCE: "<< distance << "\tANGLE_1: " << first_angle_difference);
                (*particle_itr).final_angle_diff_score_component = first_angle_diff_score;
            }

            ///Augusto: TEST FINAL. ** BE CAREFUL, THIS IS EQUAL TO INVERTED DIRECTION IF TEST2 IS ENABLED .
            transform.setOrigin( tf_pose_local_map_frame.getOrigin());
            transform.setRotation(tf_pose_local_map_frame.getRotation()*street_direction.inverse());
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_map", "tf_pose_local_map_frame_ROTATED"));

            // set particle score
            //(*particle_itr).setParticleScore(street_distribution_weight/tot_weight * pose_diff_score_component + angle_distribution_weight/tot_weight * angle_diff_score_component);
            //(*particle_itr).setParticleScore(street_distribution_weight * pose_diff_score_component * angle_distribution_weight * final_angle_diff_score);

            // vector<LayoutComponent*> vec = (*particle_itr).getLayoutComponents();
            // for(int j=0; j<vec.size(); j++)
            // {
            //     LayoutComponent* lc = vec.at(j);
            //     lc->getComponentWeight();
            //     ROS_ERROR_STREAM("PARTICLE SCORE: " << (*particle_itr).getParticleScore() << "\t-log: " << -log(abs((*particle_itr).getParticleScore()))) ;
            //     ROS_ERROR_STREAM("WE WEEE componentWeight: " << lc->getComponentWeight()<< "\t-log: "   << -log(abs(lc->getComponentWeight()))) ;
            //
            // }

            //cout << std::setprecision(5) << "PARTICLE ID: " << (*particle_itr).getId() << endl
            //     << "  SCORE:" << endl
            //     << "   DISTANCE:" << endl
            //     << "      sigma: " << street_distribution_sigma << endl
            //     << "      error: " << distance << endl
            //     << "      score: " << pose_diff_score_component << endl
            //     << "   ANGLE:" << endl
            //     << "      sigma: " << angle_distribution_sigma << endl
            //     << "     error1: " << first_angle_difference << " \tscore: " << first_angle_diff_score << endl
            //     << "     error2: " << second_angle_difference <<" \tscore: " << second_angle_diff_score <<  endl
            //     << "  sel error: " << street_direction.inverse().getAngle()<< endl
            //     << "      score: " << final_angle_diff_score << endl
            //     << "FINAL SCORE: " << (*particle_itr).getParticleScore() << endl;

        }
        else
        {
            // Either service is down or particle is too far from a street
            (*particle_itr).setParticleScore(0);
            ROS_ERROR_STREAM("RLE Main loop, snap_particle_xy service call");
            ROS_ERROR_STREAM("Either service is down or particle is too far from a street. Shutdown in LayoutManager.cpp");
            ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
        }// end snap particle client
    } // end scoring function with openstreetmap enabled
}



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
void LayoutManager::calculateScore(Particle *particle_itr) //const reference
{
    ROS_DEBUG_STREAM("Entering calculateScore() -- particleId" << particle_itr->getId());

    //(*particle_itr).setParticleScore(street_distribution_weight * (*particle_itr).pose_diff_score_component * angle_distribution_weight * (*particle_itr).final_angle_diff_score);


    vector<LayoutComponent*> vec = (*particle_itr).getLayoutComponents();
    LayoutComponent* lc = vec.at(0); //JUST BECAUSE NOW WE HAVE ONLY ONE COMPONENT

    //for(int j=0; j<vec.size(); j++)
    //{
    //    LayoutComponent* lc = vec.at(j);
    //    lc->getComponentWeight();
    //
    //}

    double debug=(  ( log(abs(street_distribution_weight    * (*particle_itr).pose_diff_score_component ))) +
                    ( log(abs(angle_distribution_weight     * (*particle_itr).final_angle_diff_score_component    ))) +
                    ( log(abs(roadState_distribution_weight * lc->getComponentWeight())))
                 );

    (*particle_itr).setParticleScore( exp(
                                            ( log(abs(street_distribution_weight    * (*particle_itr).pose_diff_score_component ))) +
                                            ( log(abs(angle_distribution_weight     * (*particle_itr).final_angle_diff_score_component    ))) +
                                            ( log(abs(roadState_distribution_weight * lc->getComponentWeight())))
                                         )
                                    );

    ROS_DEBUG_STREAM("PARTICLE SCORE DIST: \t" << std::fixed <<  (*particle_itr).pose_diff_score_component             << "\texp(log): " << exp(log(abs((*particle_itr).pose_diff_score_component)))) ;
    ROS_DEBUG_STREAM("PARTICLE SCORE ANGL: \t" << std::fixed <<  (*particle_itr).final_angle_diff_score_component      << "\texp(log): " << exp(log(abs((*particle_itr).final_angle_diff_score_component   )))) ;
    ROS_DEBUG_STREAM("COMPONENT SCORE:     \t" << std::fixed <<  lc->getComponentWeight()                              << "\texp(log): " << exp(log(abs(lc->getComponentWeight())))                 ) ;
    ROS_DEBUG_STREAM("ALL SCORES SUM :     \t" << std::fixed <<  debug                                                 << "\texp(log): " << exp(debug                 )) ;
    ROS_DEBUG_STREAM("PARTICLE SCORE TOTAL \t" << std::fixed <<  (*particle_itr).getParticleScore());

    ROS_DEBUG_STREAM("Exiting calculateScore()");

    /// OLD DARIO IMPLEMENTATION
    //vector<Particle>::iterator p_itr
    //for( p_itr = current_layout.begin(); p_itr != current_layout.end(); p_itr++ )
    //{
    //
    //    // calculate summatory of all components weights
    //    double components_weight = 0;
    //    vector<LayoutComponent*> vec = p_itr->getLayoutComponents();
    //    vector<LayoutComponent*>::iterator pc_itr;
    //    for(pc_itr = vec.begin(); pc_itr != vec.end(); pc_itr++){
    //        components_weight += (*pc_itr)->getComponentWeight();
    //    }
    //
    //    // calculate unary score
    //    MatrixXd unary_score = p_itr->getKalmanGain();
    //
    //    /// unary score con distanza dal segmento piu vicino
    //    //cout << "Particle ID: " << p_itr->getId() << ", unary score: "<< unary_score << endl;
    //
    //    // calculate binary score
    //    MatrixXd binary_score = MatrixXd::Zero(12,12);
    //
    //    // calculate particle score
    //    MatrixXd particle_score = unary_score * binary_score;
    //    //p_itr->setParticleScore(particle_score);
    //}
}
/** **************************************************************************************************************/


void LayoutManager::rleStart()
{
    LayoutManager::RLE_timer_loop.start();
}

void LayoutManager::rleStop()
{
    LayoutManager::RLE_timer_loop.stop();
}

bool LayoutManager::getAllParticlesLatLonService(road_layout_estimation::getAllParticlesLatLon::Request &req, road_layout_estimation::getAllParticlesLatLon::Response &resp)
{
    ROS_INFO_STREAM("Answering to getAllParticlesLatLonService");
    ROS_DEBUG_STREAM("> Entering getAllParticlesLatLonService");

    vector<Particle>::iterator particle_itr;
    std::vector<double> latitudes;
    std::vector<double> longitudes;
    std::vector<double> particleScores;
    std::vector<long int> way_ids;
    unsigned int particle_counter=0;

    for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ )
    {
        vector<LayoutComponent*> componentVector = (*particle_itr).getLayoutComponents();
        int components = componentVector.size();

        ROS_DEBUG_STREAM("PARTICLE " << particle_counter++ << "\tComponents Vector Size: " << components);

        for (int i=0;i<components;i++)
        {
            LayoutComponent *component = componentVector.at(i);
            if(LayoutComponent_RoadState* roadState = dynamic_cast<LayoutComponent_RoadState* >(component))
            {
                ROS_DEBUG_STREAM("ParticleId: " << (*particle_itr).getId() << "\twayId: " << roadState->getWay_id());
                way_ids.push_back(roadState->getWay_id());
                particleScores.push_back((*particle_itr).getParticleScore());

                // TRANSFORM AVERAGE POSE TO LAT/LON (NEED CONVERSION FROM LOCAL_MAP TO MAP AND ROS-SERVICE CALL)
                tf::Stamped<tf::Pose> particle_map_frame, particle_local_frame;
                particle_local_frame.frame_id_="local_map";
                particle_local_frame.setOrigin(tf::Vector3((*particle_itr).getParticleState().getPose()(0),(*particle_itr).getParticleState().getPose()(1),(*particle_itr).getParticleState().getPose()(2)));
                particle_local_frame.setRotation(tf::createIdentityQuaternion());
                // Transform pose from "local_map" to "map"
                try
                {
                    tf_listener.transformPose("map", ros::Time(0), particle_local_frame, "local_map", particle_map_frame);
                    ira_open_street_map::xy_2_latlon xy2latlon;
                    xy2latlon.request.x=particle_map_frame.getOrigin().getX();
                    xy2latlon.request.y=particle_map_frame.getOrigin().getY();
                    if (LayoutManager::xy_2_latlon_client.call(xy2latlon))
                    {
                        latitudes.push_back(xy2latlon.response.latitude);
                        longitudes.push_back(xy2latlon.response.longitude);
                    }
                    else
                    {
                        ROS_ERROR_STREAM("getAllParticlesLatLonService: Failed to call 'xy_2_latlon_2_srv' service");
                        return false;
                    }
                }
                catch (tf::TransformException &ex)
                {
                    ROS_ERROR_STREAM("getAllParticlesLatLonService failed to trasform needed poses %s"<<ex.what());
                    ROS_DEBUG_STREAM("> Exiting getAllParticlesLatLonService");
                    return false;
                }

            }
            else
            {
                ROS_ERROR_STREAM("getAllParticlesLatLonService Can't Cast");
                ROS_DEBUG_STREAM("> Exiting getAllParticlesLatLonService");
                return false;
            }
        }

    }
    resp.particleNumber=current_layout.size();
    resp.latitudes=latitudes;
    resp.longitudes=longitudes;
    resp.way_ids=way_ids;
    resp.particleScores=particleScores;

    ROS_DEBUG_STREAM("> Exiting getAllParticlesLatLonService");
    return true;
}


void LayoutManager::layoutEstimation(const ros::TimerEvent& timerEvent)
{

    ROS_INFO_STREAM("--------------------------------------------------------------------------------");
    this->deltaTimerTime = (timerEvent.current_real-timerEvent.last_real).toSec();

    ROS_INFO_STREAM ("Entering RLE layoutEstimation\t" << timerEvent.current_expected << "\t"
                                                       << timerEvent.current_real << "\t"
                                                       << timerEvent.profile.last_duration.toSec() << "\t"
                                                       << this->deltaTimerTime << "\t"
                     );



    // This should be unnecessary with the EKF enabled since we integrate the measure into the reading, but now
    // we're simply applying error as an odometry-model.
    if (this->measurement_model->measurementModelFirstRunNotExecuted)
    {
        ROS_WARN_STREAM("Still no odometry messages received! ");
        return;
    }
    else
    {
        if (layoutManagerFirstRun)
        {
            ROS_DEBUG_STREAM("measurementModelFirstRunNotExecuted: " << this->measurement_model->measurementModelFirstRunNotExecuted);
            //In the very first iteration, use the delta_time from the measurementModel
            this->deltaTimerTime=this->measurement_model->getDelta_time().toSec();
            ROS_WARN_STREAM("FIRST run! Using measurementModel delta_time for this iteration only: " << this->deltaTimerTime);

            vector<Particle>::iterator particle_itr;
            for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ )
            {
                ROS_DEBUG_STREAM("FIRST run! Setting speeds of particle n# " << (*particle_itr).getId());
                //(*particle_itr).setParticleVelocities(measurement_model->getMeasureDeltaState());             //3D Initialization
                (*particle_itr).setParticleVelocities(measurement_model->getOrthogonalMeasureDeltaState());     //do not use 3D rotations for initialization
            }

            this->layoutManagerFirstRun=false;
        }
    }

    if (this->deltaTimerTime>10)
    {
        ROS_WARN_STREAM("Warning, deltaTimer>10 seconds. Force deltatimer = 0.1 HARDCODED");
        this->deltaTimerTime = 0.1;
    }

    // Check if car has moved, if it has moved then estimate new layout
    if( checkHasMoved() )
    {
        /// ----------------- predict and update layout poses using E.K.F ----------------- //
        vector<Particle>::iterator particle_itr;
        for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ )
        {
            ROS_DEBUG_STREAM("Estimating pose of particle n# " << (*particle_itr).getId());
            (*particle_itr).particlePoseEstimation(measurement_model,this->deltaTimerTime,this->deltaOdomTime);
        }        

        // -------------- sampling + perturbation + weight layout-components ------------- //
        this->componentsEstimation();
        // ------------------------------------------------------------------------------- //

        // ------------------------------ calculate score -------------------------------- //
        /// GET SOME GEOMETRIC VALUES
        for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ )
            this->calculateGeometricScores(&(*particle_itr));

        /// COMPUTE NORMALIZATON (disabled, I just print some values here)
        double minDistance = std::numeric_limits<double>::max();
        double minAngle    = std::numeric_limits<double>::max();
        double maxDistance = std::numeric_limits<double>::min();
        double maxAngle    = std::numeric_limits<double>::min();
        for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ )
        {
            ROS_INFO_STREAM("#distance: " << (*particle_itr).pose_diff_score_component <<
                            "  \tangle (rad): " << (*particle_itr).final_angle_diff_score_component <<
                            "  \tcomponent(0): " << (*particle_itr).getLayoutComponents().at(0)->getComponentWeight()
                            );

            if ((*particle_itr).pose_diff_score_component > maxDistance)
                maxDistance=(*particle_itr).pose_diff_score_component ;
            if ((*particle_itr).pose_diff_score_component < minDistance)
                minDistance=(*particle_itr).pose_diff_score_component ;
            if ((*particle_itr).final_angle_diff_score_component > maxAngle)
                maxAngle=(*particle_itr).final_angle_diff_score_component;
            if ((*particle_itr).final_angle_diff_score_component < minAngle)
                minAngle=(*particle_itr).final_angle_diff_score_component;
        }
        ROS_DEBUG_STREAM("**********************************************************************************");
        ROS_DEBUG_STREAM("minDistance: " << minDistance << "\tmaxDistance: " << maxDistance);
        ROS_DEBUG_STREAM("minAngle   : " << minAngle    << "\tmaxAngle   : " << maxAngle);
        ROS_DEBUG_STREAM("**********************************************************************************");
        //for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ )
        //{
        //    ROS_INFO_STREAM("Before normalization\t" << (*particle_itr).pose_diff_score_component << " \tAngle: " << (*particle_itr).final_angle_diff_score_component);
        //    (*particle_itr).pose_diff_score_component = ((*particle_itr).pose_diff_score_component - minDistance )/(maxDistance-minDistance);
        //    (*particle_itr).final_angle_diff_score_component    = ((*particle_itr).final_angle_diff_score_component    - minAngle    )/(maxAngle   -minAngle   );
        //    ROS_INFO_STREAM("Normalized distance \t" << (*particle_itr).pose_diff_score_component << " \tAngle: " << (*particle_itr).final_angle_diff_score_component << endl);
        //}

        /// CALCULATE SCORE
        Particle *bestParticle   = NULL;
        double bestParticleScore = 0.0f;
        double currentScore      = 0.0f;
        for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ )
        {
            this->calculateScore(&(*particle_itr));                             //address of the particle (indicated by the vector pointer)
            currentScore = (*particle_itr).getParticleScore();

            if (currentScore>bestParticleScore)
            {
                bestParticleScore=(*particle_itr).getParticleScore();
                bestParticle = &(*(particle_itr));
            }
        }

        try
        {
            if (bestParticle != NULL)
            {
                road_layout_estimation::msg_debugInformation debugInfoMessage;
                vector<LayoutComponent*> *vec = bestParticle->getLayoutComponentsPtr();
                LayoutComponent_RoadState* lc  = dynamic_cast<LayoutComponent_RoadState*>(vec->at(0)); //JUST BECAUSE NOW WE HAVE ONLY ONE COMPONENT
                debugInfoMessage.scoreRoadLane_Lanes  = lc->scoreLanes;
                debugInfoMessage.scoreRoadLane_Width  = lc->scoreWidth;
                debugInfoMessage.scoreRoadLane_Total  = lc->totalComponentScore;
                debugInfoMessage.distance_Euclidean   = (*bestParticle).pose_diff_score_component;
                debugInfoMessage.distance_Angular     = (*bestParticle).final_angle_diff_score_component;
                debugInfoMessage.overaAllParticleScore= (*bestParticle).getParticleScore();

                this->publisher_debugInformation.publish(debugInfoMessage);
            }
        }
        catch (...)
        {
            ROS_ERROR_STREAM("Something went wrong");
        }



        // ------------------------------------------------------------------------------- //

        // ------------------------------ resampling ------------------------------------- //
        if(new_detections)
        {
//			Add new candidate-components ----------------------------------------------------- //
//			(1) given N new candidate-layout-components duplicate 2^N particles and those particles to them
//			(2) calculate score
//			(3) resample all combination
        }


        // RESAMPLING --------------------------------------------------------------------------------------------------------------------------
        if(resampling_count++ == resampling_interval)
        //    if(0)
        {
            ROS_DEBUG_STREAM("Resampling phase!");
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
                    if(uniform_rand2(rng) <= 95) //This percentage of weighted samples
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

                // copy resampled particle-set
                current_layout.clear();
                current_layout = new_current_layout;
            }
        }
        // END RESAMPLING ----------------------------------------------------------------------------------------------------------------------

        /// Other
        LayoutManager::publishMarkerArray();

    }
    else
    {
        cout << endl <<  "Not moved!" << endl;
        //TODO: calculate score again, with new detections (if available)

    }

    ROS_INFO_STREAM ("Exiting RLE layoutEstimation\t");

    //return current_layout; // current layout is the <vector> of Particles
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


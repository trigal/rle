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

bool    LayoutManager::openstreetmap_enabled = true;     ///< Check this flag if we want to initialize particle-set with OSM and GPS
double  LayoutManager::deltaOdomTime = 1;                ///< Initialize static member value for C++ compilation
double  LayoutManager::deltaTimerTime = 0.1;             ///< Initialize static member value for C++ compilation
bool    LayoutManager::layoutManagerFirstRun = true;     ///< Flag used for initiliazing particle-set with gps
bool    LayoutManager::first_msg = true;                 ///< First odometry msg flag
int     LayoutManager::odometryMessageCounter = 0;       ///< Filter step counter

visualization_msgs::Marker marker1;
visualization_msgs::Marker marker2;

inline void latlon2xy_helper(double &lat, double &lngd);

///
/// \brief LayoutManager::buildPoseArrayMsg
///        Creates a geometry message poseArray to visualize the particle pose into RVIZ
///
/// \param particles
/// \return geometry_msgs::PoseArray
///
geometry_msgs::PoseArray LayoutManager::buildPoseArrayMsg(std::vector<shared_ptr<Particle>>& particles)
{
    // init array_msg
    geometry_msgs::PoseArray array_msg;
    array_msg.header.frame_id = "local_map";

    // Insert all particles inside msg
    for (int i = 0; i < particles.size(); i++)
    {
        // build Pose from Particle
        shared_ptr<Particle> p = particles.at(i);
        geometry_msgs::Pose pose = p->getParticleState().toGeometryMsgPose();

        // normalize quaternion
        tf::Quaternion q;
        tf::quaternionMsgToTF(pose.orientation, q);
        q = q.normalize();
        tf::quaternionTFToMsg(q, pose.orientation);

        // push it!
        array_msg.poses.push_back( pose );
    }

    return array_msg;
}

/**
 * @brief LayoutManager::LayoutManager
 * @param n ROS node handler
 * @param visualOdometryTopic - up to now, we use LIBVISO2
 * @param bagfile - name of the file/map to use
 * @param timerInterval RLE frequency (parameter in the launch file)
 * @param loggingLevel ROS console level for logs
 *
 * Default constructor:
 *    1. set this node_handle as the same of 'road_layout_manager'
 *
 * TODO: bagfile is still needed even in the case of kittiplayer
 */
LayoutManager::LayoutManager(ros::NodeHandle& node_handler_parameter, std::string& visualOdometryTopic, string &bagfile, double timerInterval, ros::console::Level loggingLevel): node_handle(node_handler_parameter)
{

    // init output files
    LIBVISO_out_file.open   ("/home/cattaneod/Desktop/LIBVISO_distance.txt");
    RLE_out_file.open       ("/home/cattaneod/Desktop/RLE_distance.txt");
    RTK_GPS_out_file.open   ("/home/cattaneod/Desktop/RTK_distance.txt");

    //RLE_out_file.open("/home/cattaneod/RLE_distance.txt", ios::trunc);
    //RTK_GPS_out_file.open("/home/cattaneod/RTK_distance.txt", ios::trunc);

    /// This sets the logger level; use this to disable all ROS prints
    if ( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, loggingLevel) )
        ros::console::notifyLoggerLevelsChanged();
    else
        std::cout << "Error while setting the logger level!" << std::endl;

    /// Init values
    odometryMessageCounter = 0;
    num_particles = 0;
    resampling_count = 0;
    LayoutManager::first_msg = true;
    visualOdometryOldMsg.header.stamp = ros::Time::now();    // init header timestamp
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
    fixed_transform.setOrigin(tf::Vector3(0.0f, 0.0f, 0.0f));
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


    /// Init publisher & subscribers [RLE COMPONENTS]
    /// With this lines the components callbacks are activated/deactivated.
    /// The enabler variables are stored in LayoutManager.h
    if (componentEnabled_OSMDistance)   LayoutManager::odometry_sub  = node_handle.subscribe(visualOdometryTopic, 1, &LayoutManager::odometryCallback, this);
    //if (componentEnabled_RoadLane)      LayoutManager::road_lane_sub = node_handle.subscribe("/kitti_player/lanes", 3, &LayoutManager::roadLaneCallback , this); //ISISLAB Ruben callback   //#573 - safe state
    //if (componentEnabled_RoadState)     LayoutManager::roadState_sub = node_handle.subscribe("/kitti_player/lanes", 3, &LayoutManager::roadStateCallback, this);                            //#573 - safe state
    if (componentEnabled_RoadLane)      LayoutManager::road_lane_sub = node_handle.subscribe("/isis_line_detector/lines", 3, &LayoutManager::roadLaneCallback , this); //ISISLAB Ruben callback   //#573 - safe state
    if (componentEnabled_RoadState)     LayoutManager::roadState_sub = node_handle.subscribe("/isis_line_detector/lines", 3, &LayoutManager::roadStateCallback, this);                            //#573 - safe state
    if (componentEnabled_Building)      LayoutManager::buildings_sub = node_handle.subscribe("/building_detector/facades", 1, &LayoutManager::buildingsCallback, this);
    //LayoutManager::roadState_sub = node_handle.subscribe("/fakeDetector/roadState"   , 3, &LayoutManager::roadStateCallback, this);  //fake detector

    ROS_INFO_STREAM("RLE started, listening to: " << odometry_sub.getTopic());

    // Publisher Section
    LayoutManager::array_pub                            = node_handler_parameter.advertise<geometry_msgs::PoseArray>              ("/road_layout_estimation/layout_manager/particle_pose_array", 1);
    LayoutManager::gps_pub                              = node_handler_parameter.advertise<sensor_msgs::NavSatFix>                ("/road_layout_estimation/layout_manager/gps_fix", 1);
    LayoutManager::street_publisher                     = node_handler_parameter.advertise<geometry_msgs::PoseStamped>            ("/road_layout_estimation/layout_manager/quaternion_pose", 1);
    LayoutManager::particle_publisher                   = node_handler_parameter.advertise<geometry_msgs::PoseStamped>            ("/road_layout_estimation/layout_manager/particle_pose", 1);
    LayoutManager::diff_publisher                       = node_handler_parameter.advertise<geometry_msgs::PoseStamped>            ("/road_layout_estimation/layout_manager/diff_pose", 1);
    LayoutManager::marker_pub                           = node_handler_parameter.advertise<visualization_msgs::Marker>            ("/road_layout_estimation/layout_manager/circle", 1);
    LayoutManager::marker_pub2                          = node_handler_parameter.advertise<visualization_msgs::Marker>            ("/road_layout_estimation/layout_manager/circle2", 1);
    LayoutManager::publisher_marker_array               = node_handler_parameter.advertise<visualization_msgs::MarkerArray>       ("/road_layout_estimation/layout_manager/particle_set", 1);
    LayoutManager::publisher_marker_array_distances     = node_handler_parameter.advertise<visualization_msgs::MarkerArray>       ("/road_layout_estimation/layout_manager/marker_array_distances", 1);
    LayoutManager::publisher_marker_array_angles        = node_handler_parameter.advertise<visualization_msgs::MarkerArray>       ("/road_layout_estimation/layout_manager/publisher_marker_array_angles", 1);
    LayoutManager::publisher_z_particle                 = node_handler_parameter.advertise<visualization_msgs::MarkerArray>       ("/road_layout_estimation/layout_manager/z_particle", 1);
    LayoutManager::publisher_z_snapped                  = node_handler_parameter.advertise<visualization_msgs::MarkerArray>       ("/road_layout_estimation/layout_manager/z_snapped", 1);
    LayoutManager::publisher_GT_RTK                     = node_handler_parameter.advertise<visualization_msgs::MarkerArray>       ("/road_layout_estimation/layout_manager/GT_RTK", 1);
    LayoutManager::publisher_average_position           = node_handler_parameter.advertise<visualization_msgs::MarkerArray>       ("/road_layout_estimation/layout_manager/average_position", 1);
    LayoutManager::publisher_average_pose               = node_handler_parameter.advertise<nav_msgs::Odometry>                    ("/road_layout_estimation/layout_manager/average_pose", 1);
    LayoutManager::RLETIME                              = node_handler_parameter.advertise<std_msgs::Int64>                        ("/road_layout_estimation/layout_manager/loop_time", 1);

    LayoutManager::publisher_debugInformation           = node_handler_parameter.advertise<road_layout_estimation::msg_debugInformation >("/road_layout_estimation/debugInformation", 1);
    LayoutManager::facades_pub                          = node_handler_parameter.advertise<sensor_msgs::PointCloud2>              ("/facades_cloud_gps", 1);

    // Init ROS service clients
    latlon_2_xy_client                  = node_handler_parameter.serviceClient<ira_open_street_map::latlon_2_xy>                  ("/ira_open_street_map/latlon_2_xy");
    xy_2_latlon_client                  = node_handler_parameter.serviceClient<ira_open_street_map::xy_2_latlon>                  ("/ira_open_street_map/xy_2_latlon");
    snap_particle_xy_client             = node_handler_parameter.serviceClient<ira_open_street_map::snap_particle_xy>             ("/ira_open_street_map/snap_particle_xy");
    get_closest_way_distance_utm_client = node_handler_parameter.serviceClient<ira_open_street_map::get_closest_way_distance_utm> ("/ira_open_street_map/get_closest_way_distance_utm");
    getHighwayInfo_client               = node_handler_parameter.serviceClient<ira_open_street_map::getHighwayInfo>               ("/ira_open_street_map/getHighwayInfo");
    getDistanceFromLaneCenter_client    = node_handler_parameter.serviceClient<ira_open_street_map::getDistanceFromLaneCenter>    ("/ira_open_street_map/getDistanceFromLaneCenter");
    oneWay_client                       = node_handler_parameter.serviceClient<ira_open_street_map::oneway>                       ("/ira_open_street_map/oneWay");

    // Init ROS service server
    server_getAllParticlesLatLon        = node_handler_parameter.advertiseService("/road_layout_estimation/layout_manager/getAllParticlesLatLon" , &LayoutManager::getAllParticlesLatLonService, this);


    // RLE system initialization
    latlon_2_xy_client.waitForExistence();      // WAIT FOR SERVICE -- the function prints some pretty comments

    deltaTimerTime = timerInterval;             ///< deltaTimer of the LayoutManager Class is initialy set as the requested interval
    RLE_timer_loop = node_handler_parameter.createTimer(ros::Duration(deltaTimerTime), &LayoutManager::layoutEstimation, this, false, false);

    /// For debug purposes, print default paramenters (from .launch or .cfg file)
    ROS_DEBUG_STREAM("propagate_translational_absolute_vel_error_x    ------------  " << currentLayoutManagerConfiguration.propagate_translational_absolute_vel_error_x  );
    ROS_DEBUG_STREAM("propagate_translational_absolute_vel_error_y    ------------  " << currentLayoutManagerConfiguration.propagate_translational_absolute_vel_error_y  );
    ROS_DEBUG_STREAM("propagate_translational_absolute_vel_error_z    ------------  " << currentLayoutManagerConfiguration.propagate_translational_absolute_vel_error_z  );
    ROS_DEBUG_STREAM("propagate_rotational_absolute_vel_error         ------------  " << currentLayoutManagerConfiguration.propagate_rotational_absolute_vel_error       );
    ROS_DEBUG_STREAM("propagate_translational_percentage_vel_error_x  ------------  " << currentLayoutManagerConfiguration.propagate_translational_percentage_vel_error_x);
    ROS_DEBUG_STREAM("propagate_translational_percentage_vel_error_y  ------------  " << currentLayoutManagerConfiguration.propagate_translational_percentage_vel_error_y);
    ROS_DEBUG_STREAM("propagate_translational_percentage_vel_error_z  ------------  " << currentLayoutManagerConfiguration.propagate_translational_percentage_vel_error_z);
    ROS_DEBUG_STREAM("propagate_rotational_percentage_vel_error       ------------  " << currentLayoutManagerConfiguration.propagate_rotational_percentage_vel_error     );

    //nodelet::M_string   remappings;
    //nodelet::V_string   my_argv;
    //manager = new nodelet::Loader(node_handler_parameter);
    //manager.load("isis_line_detector" ,"isis_line_detector/get_principal_plane"   ,remappings,my_argv);
    //manager.load("lineTracker"        ,"isis_line_detector/lineTracker"           ,remappings,my_argv);


    // Finally, setup the reconfigureCallback. This will initialize the components.
    dynamicReconfigureCallback = boost::bind(&LayoutManager::reconfigureCallback, this, _1, _2);
    dynamicReconfigureServer.setCallback(dynamicReconfigureCallback);
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
    tmp_marker1.scale.x = sqrt(cov1) * 3 * 2; //*2 Scale of the marker. Applied before the position/orientation. A scale of [1,1,1] means the object will be 1m by 1m by 1m.
    tmp_marker1.scale.y = sqrt(cov2) * 3 * 2; //*2 Scale of the marker. Applied before the position/orientation. A scale of [1,1,1] means the object will be 1m by 1m by 1m.
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

    tf_pose_map_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf_pose_map_frame.setRotation(tf::createIdentityQuaternion());

    // Transform pose from "local_map" to "map"
    try
    {
        tf_listener.transformPose("map", ros::Time(0), tf_pose_local_map_frame, "local_map", tf_pose_map_frame);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ROS_INFO_STREAM("if(!LayoutManager::first_run){ if(config.particles_number > num_particles)");
        ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
        return tf_pose_map_frame;
    }

    return tf_pose_map_frame;
}
double LayoutManager::getCurrent_layoutScore() const
{
    return current_layoutScore;
}

void LayoutManager::setCurrent_layoutScore(double value)
{
    current_layoutScore = value;
}


/**
 * @brief LayoutManager::reconfigureCallback
 * @param config    same as constr
 * @param level
 *
 * This modifies the number of particles and other parameters from the Dynamic
 * Reconfigure Node
 *
 * This callback is also called during the creation of the RLE node by the
 * constructor (FIRST RUN section) and in this case
 *      1. creates only valid particles (near the road segments)
 *      2. creates opposite particles if the way is two-ways
 *      3. creates component > ROAD_STATE
 *                 component > ROAD_LANE
 *
 */
void LayoutManager::reconfigureCallback(road_layout_estimation::road_layout_estimationConfig &currentConfiguration, uint32_t level)
{
    ROS_INFO_STREAM ("Reconfigure callback! " << bagfile );

    // Save the configuration inside the LayoutManager #fixes 533
    currentLayoutManagerConfiguration = currentConfiguration;

    // update score gaussian distribution values
    street_distribution_sigma     = currentConfiguration.street_distribution_sigma;         // #522 to be deleted
    angle_distribution_sigma      = currentConfiguration.angle_distribution_sigma;          // #522 to be deleted
    street_distribution_alpha     = currentConfiguration.street_distribution_alpha;         // #522 to be deleted
    angle_distribution_alpha      = currentConfiguration.angle_distribution_alpha;          // #522 to be deleted

    roadState_distribution_alpha  = currentConfiguration.roadState_distribution_alpha;      // #534  refactor weight>alpha
    resampling_interval           = currentConfiguration.resampling_interval;

    // update uncertainty values -----------------------------------------------------------------------------------
    for (int i = 0; i < current_layout_shared.size(); ++i)
    {
        //Particle* particle_ptr = &current_layout.at(i);
        shared_ptr<Particle> particle_ptr = current_layout_shared.at(i);

        MotionModel* motionModelPointer = particle_ptr->getMotionModelPtr();

        ///disabling
        //motionModelPointer->setErrorCovariance(
        //            config.mtn_model_position_uncertainty,
        //            config.mtn_model_orientation_uncertainty,
        //            config.mtn_model_linear_uncertainty,
        //            config.mtn_model_angular_uncertainty
        //            );

        motionModelPointer->setPropagationError(
            currentConfiguration.propagate_translational_absolute_vel_error_x,
            currentConfiguration.propagate_translational_absolute_vel_error_y,
            currentConfiguration.propagate_translational_absolute_vel_error_z,
            currentConfiguration.propagate_rotational_absolute_vel_error,
            currentConfiguration.propagate_translational_percentage_vel_error_x,
            currentConfiguration.propagate_translational_percentage_vel_error_y,
            currentConfiguration.propagate_translational_percentage_vel_error_z,
            currentConfiguration.propagate_rotational_percentage_vel_error
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
        currentConfiguration.propagate_translational_absolute_vel_error_x,
        currentConfiguration.propagate_translational_absolute_vel_error_y,
        currentConfiguration.propagate_translational_absolute_vel_error_z,
        currentConfiguration.propagate_rotational_absolute_vel_error,
        currentConfiguration.propagate_translational_percentage_vel_error_x,
        currentConfiguration.propagate_translational_percentage_vel_error_y,
        currentConfiguration.propagate_translational_percentage_vel_error_z,
        currentConfiguration.propagate_rotational_percentage_vel_error
    );

    ///disabling
    // TODO: verificare. perché c'è questa cosa? Non dovrebbe essere letta dal messaggio di odometria? Questa è Q in EKF.
    //measurement_model->setMeasureCov(
    //            config.msr_model_position_uncertainty,
    //            config.msr_model_orientation_uncertainty,
    //            config.msr_model_linear_uncertainty,
    //            config.msr_model_angular_uncertainty
    //            );



    /*
     * BUG: #530 the following IF-CLAUSE is bugged, see the task on redmine.
     * Luckly it does nothing untile someone change the number of particles
     * using the dynamic reconfigure, but sooner or later a fix is needed
     */
    ////////////////////////////////////////////////////////////////////////////
    /// update particle-set number (only if this IS NOT the first run)       ///
    ////////////////////////////////////////////////////////////////////////////
    if (!LayoutManager::layoutManagerFirstRun)
    {
        if (currentConfiguration.particles_number > num_particles)
        {
            // let's add some empty particles to particle-set:
            int counter = current_layout_shared.size(); //this will keep track of current ID
            int particles_to_add = currentConfiguration.particles_number - num_particles;
            for (int i = 0; i < particles_to_add; i++)
            {
                ROS_WARN_STREAM("NEW PARTICLE");
                //Particle new_particle(node_handle, counter, default_mtn_model);
                shared_ptr<Particle> newParticlePtr = make_shared<Particle>(node_handle, &tf_listener, counter, default_mtn_model); //refs #616

                newParticlePtr->in_cluster = -1;
                //#522                newParticlePtr->distance_to_closest_segment = 0.0f; //default value
                // BUG: this is the old behavior with the distance stored as a particle parameter
                State6DOF tmp;
                tmp.addNoise(0.5, 0.5, 0.5, 0.5);

                newParticlePtr->setParticleState(tmp); // BUG #530

                // update particle score using OpenStreetMap
                if (LayoutManager::openstreetmap_enabled)
                {

                    // Build request
                    ira_open_street_map::snap_particle_xy snapParticleRequestResponse;

                    tf::Stamped<tf::Pose> tf_pose_map_frame = toGlobalFrame(newParticlePtr->getParticleState().getPosition());
                    snapParticleRequestResponse.request.x = tf_pose_map_frame.getOrigin().getX();
                    snapParticleRequestResponse.request.y = tf_pose_map_frame.getOrigin().getY();
                    snapParticleRequestResponse.request.max_distance_radius = 100; // distance radius for finding the closest nodes for particle snap

                    // Get distance from closest street and set it as particle score
                    // init normal distribution
                    boost::math::normal normal_dist(0, currentConfiguration.street_distribution_sigma);
                    if (LayoutManager::snap_particle_xy_client.call(snapParticleRequestResponse))
                    {
                        // calculate difference between original particle position and snapped particle position
                        // use it to set particle score using normal distribution PDF
                        double dx = tf_pose_map_frame.getOrigin().getX() - snapParticleRequestResponse.response.snapped_x;
                        double dy = tf_pose_map_frame.getOrigin().getY() - snapParticleRequestResponse.response.snapped_y;
                        double dz = tf_pose_map_frame.getOrigin().getZ() - 0;
                        double distance = sqrt(dx * dx + dy * dy + dz * dz);
                        //#522                        newParticlePtr->distance_to_closest_segment = distance;
                        // BUG: this is the old behavior with the distance stored as a particle parameter
                        newParticlePtr->setParticleScore(pdf(normal_dist, distance));
                    }
                    else
                    {
                        // Either service is down or particle is too far from a street
                        newParticlePtr->setParticleScore(0);
                    }
                }

                // Push particle
                current_layout_shared.push_back(newParticlePtr);

                // Update counter
                counter = counter + 1;
            }// end cycle for creating particle (FOR)
        }
        else if (currentConfiguration.particles_number < num_particles)
        {
            // let's erase particles starting from particle-set tail
            // WARNING: should we remove RANDOM particles instead of 'the last N particles'?
            int particles_to_remove = num_particles - currentConfiguration.particles_number;
            for (int i = 0; i < particles_to_remove; i++)
            {
                // delete last element
                current_layout_shared.erase(current_layout_shared.end());
            }
        }

        num_particles = currentConfiguration.particles_number;
    }



    ////////////////////////////////////////////////////////////////////////////
    /// -----FIRST RUN----------------------------------------------------------
    ////////////////////////////////////////////////////////////////////////////
    if (LayoutManager::layoutManagerFirstRun)
    {
        ROS_ASSERT(num_particles == 0); // refs #385

        if (LayoutManager::openstreetmap_enabled)
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

            } // end getting the GPS from message
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
                if (bagfile.compare("kitti_00") == 0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_00 intial coordinates");
                    alt = 0;
                    lat = 48.9825523586602;//48.98254523586602;
                    lon = 8.39036610004500; //8.39036610004500;
                    cov1 = 15;
                    cov2 = 15;
                }

                // KITTI 01 [OK, video autostrada, si perde nella curva finale]
                if (bagfile.compare("kitti_01") == 0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_01 intial coordinates");
                    alt = 0;
                    lat = 49.006719195871;//49.006558;// 49.006616;//
                    lon = 8.4893558806503;//8.489195;//8.489291;//
                    cov1 = 50;
                    cov2 = 50;
                }

                // KITTI 02 [NI, si perde dopo un paio di curve]
                if (bagfile.compare("kitti_02") == 0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_02 intial coordinates");
                    alt = 0;
                    lat = 48.987607723096;
                    lon = 8.4697469732634;
                    cov1 = 60;
                    cov2 = 60;
                }

                // KITTI 04 [OK, video road, rettilineo corto]
                if (bagfile.compare("kitti_04") == 0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_04 intial coordinates");
                    alt = 0;
                    lat = 49.033603440345;
                    lon = 8.3950031909457;
                    cov1 = 50;
                    cov2 = 50;
                }

                // KITTI 05 [NI, se imbocca la strada giusta nell'inizializzazione funziona bene]
                if (bagfile.compare("kitti_05") == 0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_05 intial coordinates");
                    lat = 49.04951961077;
                    lon = 8.3965961639946;
                    alt = 0;
                    cov1 = 4;
                    cov2 = 4;
                }

                // KITTI 06 [OK, video loop, si perde dopo il secondo incrocio]
                if (bagfile.compare("kitti_06") == 0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_06 intial coordinates");
                    alt = 0;
                    lat = 49.05349304789598;
                    lon = 8.39721998765449;
                    cov1 = 50;
                    cov2 = 50;
                }

                // KITTI 07 [CUTTED OK, video in cui sta fermo allo stop alcuni secondi]
                if (bagfile.compare("kitti_07") == 0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_07 intial coordinates");
                    alt = 0;
                    lat = 48.985319;//48.98523696217;
                    lon = 8.393801;//8.3936414564418;
                    cov1 = 50;
                    cov2 = 50;
                }

                // KITTI 08 [bag inizia dopo]
                if (bagfile.compare("kitti_08") == 0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_08 intial coordinates");
                    alt = 0;
                    lat = 48.984311;
                    lon = 8.397817;
                    cov1 = 60;
                    cov2 = 60;
                }

                // KITTI 09 [OK, video serie curve tondeggianti]
                if (bagfile.compare("kitti_09") == 0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_09 intial coordinates");
                    alt = 0;
                    lat = 48.972104544468;
                    lon = 8.4761469953335;
                    cov1 = 60;
                    cov2 = 60;
                }

                // KITTI 10 [CUTTED OK, non esegue l'inversione finale rimane indietro]
                if (bagfile.compare("kitti_10") == 0)
                {
                    ROS_INFO_STREAM("Using hard-coded kitti_10 intial coordinates");
                    alt = 0;
                    lat = 48.972406;//48.972455;//48.97253396005;
                    lon = 8.478662;//8.478660;//8.4785980847297;
                    cov1 = 50;
                    cov2 = 50;
                }
            } // end getting the lat/lon from  hard-coded kitti bags

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

            ROS_INFO_STREAM("RLE is going to initialize particles at LAT: " << lat << "\t" << "LON: " << lon);

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
            covar(0, 0) = cov1;
            covar(1, 1) = cov2;

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

            particle_poses_statistics = MatrixXd(currentConfiguration.particles_number, 2);

            // Create a bivariate gaussian distribution of doubles.
            // with our chosen mean and covariance
            Eigen::EigenMultivariateNormal<double, 2> normalSampler(mean, covar);

            /// Setep 01 - Reset current_layout_shared
            LayoutManager::current_layout_shared.clear();

            /// Setep 02 - Populate current_layout_shared with *ONLY* valid particles
            int particle_id = 1;
            int component_id = 1;
            int while_ctr = 1; // while loop infinite cycle prevenction
            while ((while_ctr < 1000) && (current_layout_shared.size() < currentConfiguration.particles_number))
            {
                if (while_ctr == 999)
                {
                    ROS_ERROR_STREAM("Random particle set init: while ctr reached max limit" );
                    ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
                    /*
                     * TODO: max_radius_size * 2 and find again
                     */
                }

                // Generate a sample from the bivariate Gaussian distribution
                Matrix < double, 2, -1 > sample = normalSampler.samples(1);

                //-------------- SNAP PARTICLE v2 -------------- //
                // Init OSM cartography service
                ira_open_street_map::snap_particle_xy snapParticleXYService;
                snapParticleXYService.request.x = sample(0);
                snapParticleXYService.request.y = sample(1);
                snapParticleXYService.request.max_distance_radius = 200; // distance radius for finding the closest nodes for particle snap
                //boost::math::normal street_normal_dist (0, street_distribution_sigma);
                //boost::math::normal angle_normal_dist  (0, angle_distribution_sigma);

                /// Setep 03 - Call snap particle service
                /// the returned particle snapped has ZERO distance from the road segment and ZERO misalignment
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
                    tf::poseStampedMsgToTF(pose_map_frame, tf_pose_map_frame);

                    // Transform pose from "map" to "local_map"
                    try
                    {
                        tf_listener.waitForTransform("local_map", "map", ros::Time(0), ros::Duration(0.5));
                        tf_listener.transformPose("/local_map", ros::Time(0), tf_pose_map_frame, "/map", tf_pose_local_map_frame);
                    }
                    catch (tf::TransformException &ex)
                    {
                        ROS_ERROR_STREAM("%s" << ex.what());
                        ROS_INFO_STREAM("map to local map exception");
                        continue; //Skip this iteration
                    }

                    // Init particle's pose
                    State6DOF p_pose;
                    p_pose.setPose(Vector3d(tf_pose_local_map_frame.getOrigin().getX(), tf_pose_local_map_frame.getOrigin().getY(), 0));

                    // Add noise to the snapped particles?! This may be useful for the stress the building detector
                    if (false) //TODO: this refs #634
                        p_pose.addNoise(1.5, 0.07, 0, 0);

                    bool ok_direction = true; //inverte direzione, occhio che c'è opposite_direction sotto
                    if (ok_direction)
                    {
                        tf_pose_local_map_frame.setRotation(tf_pose_local_map_frame.getRotation().normalized());

                        AngleAxisd rotation = AngleAxisd(
                                                  Quaterniond(
                                                      tf_pose_local_map_frame.getRotation().getW(),
                                                      tf_pose_local_map_frame.getRotation().getX(),
                                                      tf_pose_local_map_frame.getRotation().getY(),
                                                      tf_pose_local_map_frame.getRotation().getZ()
                                                  ));
                        p_pose.setRotation(rotation);
                    }
                    else
                    {

                        tf_pose_local_map_frame.setRotation(tf_pose_local_map_frame.getRotation().normalized() * tf::createQuaternionFromYaw(M_PI));

                        AngleAxisd rotation = AngleAxisd(
                                                  Quaterniond(
                                                      tf_pose_local_map_frame.getRotation().getW(),
                                                      tf_pose_local_map_frame.getRotation().getX(),
                                                      tf_pose_local_map_frame.getRotation().getY(),
                                                      tf_pose_local_map_frame.getRotation().getZ()
                                                  ));
                        p_pose.setRotation(rotation);

                    }





                    // Init particle's sigma
                    MatrixXd p_sigma = default_mtn_model.getErrorCovariance();

                    ROS_DEBUG_STREAM("p_sigma" << endl << endl << p_sigma << endl);

                    // Create particle, set its score and other parameters
                    //Particle new_particle(particle_id, p_pose, p_sigma, default_mtn_model);
                    shared_ptr <Particle> new_particle = make_shared<Particle>(node_handle, &tf_listener, particle_id, p_pose, p_sigma, default_mtn_model); //refs #616
                    //#522                    new_particle->distance_to_closest_segment = 0.0f;       //distance is ZERO because particle is snapped

                    //new_particle.setParticleScore(pdf(street_normal_dist,0) * pdf(angle_normal_dist, 0)); // dont' calculate score with distance because particle is snapped
                    new_particle->setParticleScore(1); // dont' calculate score with distance because particle is snapped
                    // WARNING: in the line above, comment says something different from what is written. furthermore, pdf are not weighted like:
                    // street_distribution_alpha * pose_diff_score_component * angle_distribution_alpha * final_angle_diff_score
                    ROS_DEBUG_STREAM("Initialized particle with score: " << new_particle->getParticleScore());



                    /// Step 04 - Create ROAD-RELATED Components
                    /// ROAD_STATE          Checks width + Lane number wrt OSM
                    /// ROAD_LANE           Checks lane displacement wrt OSM
                    /// ROAD OSM-DISTANCE   Here are stored the distances from the OSM segment once stored inside the particle
                    ira_open_street_map::getHighwayInfo getHighwayService;
                    getHighwayService.request.way_id = snapParticleXYService.response.way_id;
                    if (LayoutManager::getHighwayInfo_client.call(getHighwayService))
                    {
                        //#573 - safe state    START *****************************************************************************************************
                        road_layout_estimation::msg_lines lines;
                        road_layout_estimation::msg_lineInfo lineInfo;

                        lineInfo.isValid = false;
                        lineInfo.offset  = 0.0f;
                        lineInfo.counter = 0;

                        lines.way_id          = snapParticleXYService.response.way_id;
                        lines.width           = getHighwayService.response.width;
                        lines.goodLines       = 0;
                        lines.number_of_lines = Utils::linesFromLanes(getHighwayService.response.number_of_lanes);
                        for (int i = 0; i < getHighwayService.response.number_of_lanes; i++)
                            lines.lines.push_back(lineInfo);

                        VectorXd state = new_particle->getParticleState().getPosition();



                        /*
                         * *********** ROAD STATE COMPONENT ***********
                         * Creating the FIRST component, ROAD STATE COMPONENT
                         */
                        if (componentEnabled_RoadState)
                        {
                            ROS_INFO_STREAM("Adding roadState component!");
                            LayoutComponent_RoadState *roadState = new LayoutComponent_RoadState(particle_id,
                                                                                                 component_id++,
                                                                                                 ros::Time::now(),
                                                                                                 &this->getHighwayInfo_client,
                                                                                                 lines,
                                                                                                 getHighwayService.response.oneway,
                                                                                                 currentConfiguration.roadState_distribution_alpha);
                            roadState->setParticlePtr(new_particle); //adding the pointer to newly created particle, refs #523 -- this creates the #529 bug -- FIXED with #531
                            roadState->setComponentState(state);
                            new_particle->addComponent(roadState);
                            ROS_INFO_STREAM("road state just created: " << roadState->getComponentId() << "\t" << roadState->getComponentState()(0));
                        }


                        /*
                         * *********** ROAD LANE COMPONENT ***********
                         * Creating this second component here; this ensure that
                         * we have same number of both components in each particle
                         */
                        if (componentEnabled_RoadLane)
                        {
                            ROS_INFO_STREAM("Adding roadLane component!");
                            LayoutComponent_RoadLane *roadLane = new LayoutComponent_RoadLane(particle_id,
                                                                                              component_id++,
                                                                                              getHighwayService.response.number_of_lanes,
                                                                                              currentConfiguration.roadLane_distribution_alpha);
                            roadLane->setParticlePtr(new_particle); //adding the pointer to newly created particle, refs #523 -- this creates the #529 bug  -- FIXED with #531
                            new_particle->addComponent(roadLane);
                        }
                        //#573 - safe state    END *****************************************************************************************************


                        /*
                         * Creating the third component here: ROAD OSM DISTANCE for the two eucliedean distances we used in the RLE-ITSC2015. This refs #522
                         * The distances are ZERO since the particle was just created and snapped to the nearest road segment. Also the angular is zero, because
                         * we're saving the misalignment, that is zero in this case.
                         * Warning <<Feature #634>>
                         */
                        if (componentEnabled_OSMDistance)
                        {
                            ROS_INFO_STREAM("Adding OSM-Distance component!");
                            LayoutComponent_OSMDistance *roadOSMDistance = new LayoutComponent_OSMDistance(node_handle, &tf_listener, particle_id,
                                                                                                           component_id,
                                                                                                           0.0f,    //distance_to_closest_segment
                                                                                                           0.0f,    //final_angle_diff_score_component
                                                                                                           currentConfiguration.street_distribution_sigma,
                                                                                                           currentConfiguration.angle_distribution_sigma,
                                                                                                           currentConfiguration.street_distribution_alpha,
                                                                                                           currentConfiguration.angle_distribution_alpha
                                                                                                          );
                            roadOSMDistance->setParticlePtr(new_particle);
                            new_particle->addComponent(roadOSMDistance);
                        }

                    }
                    else
                        ROS_ERROR_STREAM("Can't add ROAD RELATED components due to getHighwayInfo call failure");
                    //////////// CREATE ROAD RELATED COMPONENTS ////////////



                    /// OTHER COMPONENTS
                    /// 1. Buildings
                    /// 2. Crossings

                    if (componentEnabled_Building)
                    {
                        ROS_INFO_STREAM("Adding Building component!");
                        LayoutComponent_Building *buildingComponent = new LayoutComponent_Building(node_handle, &tf_listener, particle_id,
                                                                                                   component_id,
                                                                                                   VectorXd::Zero(12),
                                                                                                   MatrixXd::Zero(12, 12)
                                                                                                  );
                        buildingComponent->setParticlePtr(new_particle);
                        new_particle->addComponent(buildingComponent);
                    }

                    /// Setep 05 - Push particle into particle-set and update the particles id counter
                    current_layout_shared.push_back(new_particle);



                    // Update particles id counter
                    particle_id += 1;



                    /// Setep 06 - Check if we should create 2 particles with opposite direction
                    /// if needed, create new "components"
                    if (snapParticleXYService.response.way_dir_opposite_particles)
                    {
                        tf::Stamped<tf::Pose>  tf_pose_local_map_frame_opposite_direction;
                        tf_pose_local_map_frame_opposite_direction = tf_pose_local_map_frame;
                        tf_pose_local_map_frame_opposite_direction.setRotation(tf_pose_local_map_frame * tf::createQuaternionFromYaw(M_PI)); // INVERT DIRECTION

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

                        // Create particle, set its score and other parameters
                        //Particle new_particle_opposite(particle_id, p_pose, p_sigma, default_mtn_model);
                        shared_ptr <Particle> new_particle_opposite = make_shared<Particle>(node_handle, &tf_listener, particle_id, p_pose, p_sigma, default_mtn_model); //refs #616
                        //#522                        new_particle_opposite->distance_to_closest_segment = 0.0f; //distance is ZERO because particle is snapped
                        //new_particle_opposite.setParticleScore(pdf(street_normal_dist,0) * pdf(angle_normal_dist, 0)); // don't calculate score with distance because particle is snapped
                        new_particle_opposite->setParticleScore(1); // don't calculate score with distance because particle is snapped
                        // WARNING: in the line above, comment says something different from what is written. furthermore, pdf are not weighted like:
                        // street_distribution_weight * pose_diff_score_component * angle_distribution_weight * final_angle_diff_score
                        ROS_DEBUG_STREAM("Initialized particle with score: " << new_particle->getParticleScore());


                        //////////// CREATE ROAD RELATED COMPONENTS - inside OPPOSITE DIRECTION ////////////
                        ira_open_street_map::getHighwayInfo getHighwayService;
                        getHighwayService.request.way_id = snapParticleXYService.response.way_id;
                        if (LayoutManager::getHighwayInfo_client.call(getHighwayService))
                        {

                            //#573 - safe state    START *****************************************************************************************************
                            road_layout_estimation::msg_lines lines;
                            road_layout_estimation::msg_lineInfo lineInfo;

                            lineInfo.isValid = false;
                            lineInfo.offset  = 0.0f;
                            lineInfo.counter = 0;

                            lines.way_id          = snapParticleXYService.response.way_id;
                            lines.width           = getHighwayService.response.width;
                            lines.goodLines       = 0;
                            lines.number_of_lines = Utils::linesFromLanes(getHighwayService.response.number_of_lanes);
                            for (int i = 0; i < getHighwayService.response.number_of_lanes; i++)
                                lines.lines.push_back(lineInfo);


                            VectorXd state = new_particle_opposite->getParticleState().getPosition();

                            /*
                             * *********** ROAD STATE COMPONENT ***********
                             * Creating the FIRST component, ROAD STATE COMPONENT
                             */
                            if (componentEnabled_RoadState)
                            {
                                ROS_INFO_STREAM("Adding roadState component!");
                                LayoutComponent_RoadState *roadState = new LayoutComponent_RoadState(particle_id,
                                                                                                     component_id++,
                                                                                                     ros::Time::now(),
                                                                                                     &this->getHighwayInfo_client,
                                                                                                     lines,
                                                                                                     getHighwayService.response.width,
                                                                                                     currentConfiguration.roadState_distribution_alpha);
                                roadState->setParticlePtr(new_particle_opposite); //adding the pointer to newly created particle, refs #523 -- this creates the #529 bug -- FIXED with #531
                                (*roadState).setComponentState(state);
                                new_particle_opposite->addComponent(roadState);
                                ROS_INFO_STREAM("road state just created: " << roadState->getComponentId() << "\t" << roadState->getComponentState()(0));
                            }


                            /*
                             * *********** ROAD LANE COMPONENT ***********
                             * Creating this second component here; this ensure that
                             * we have same number of both components in each particle
                             */
                            if (componentEnabled_RoadLane)
                            {
                                ROS_INFO_STREAM("Adding roadLane component!");
                                LayoutComponent_RoadLane *roadLane = new LayoutComponent_RoadLane(particle_id,
                                                                                                  component_id++,
                                                                                                  getHighwayService.response.number_of_lanes,
                                                                                                  currentConfiguration.roadLane_distribution_alpha);
                                roadLane->setParticlePtr(new_particle_opposite); //adding the pointer to newly created particle, refs #523 -- this creates the #529 bug -- FIXED with #531
                                new_particle_opposite->addComponent(roadLane);
                            }
                            //#573 - safe state    END *****************************************************************************************************


                            /*
                             * Creating the third component here: ROAD OSM DISTANCE for the two eucliedean distances we used in the RLE-ITSC2015. This refs #522
                             * The distances are ZERO since the particle was just created and snapped to the nearest road segment. Also the angular is zero, because
                             * we're saving the misalignment, that is zero in this case.
                             * Warning <<Feature #634>>
                             */
                            if (componentEnabled_OSMDistance)
                            {
                                LayoutComponent_OSMDistance *roadOSMDistance = new LayoutComponent_OSMDistance(node_handle, &tf_listener , particle_id,
                                                                                                               component_id,
                                                                                                               0.0f,    //distance_to_closest_segment
                                                                                                               0.0f,    //final_angle_diff_score_component
                                                                                                               currentConfiguration.street_distribution_sigma,
                                                                                                               currentConfiguration.angle_distribution_sigma,
                                                                                                               currentConfiguration.street_distribution_alpha,
                                                                                                               currentConfiguration.angle_distribution_alpha
                                                                                                              ); //refs #616
                                roadOSMDistance->setParticlePtr(new_particle_opposite);
                                new_particle_opposite->addComponent(roadOSMDistance);
                            }

                        }
                        else
                            ROS_ERROR_STREAM("Can't add ROAD RELATED components due to getHighwayInfo call failure");
                        //////////// CREATE ROAD RELATED COMPONENTS - inside OPPOSITE DIRECTION ////////////


                        if (componentEnabled_Building)
                        {
                            LayoutComponent_Building *buildingComponent = new LayoutComponent_Building(node_handle, &tf_listener, particle_id,
                                                                                                       component_id,
                                                                                                       VectorXd::Zero(12),
                                                                                                       MatrixXd::Zero(12, 12)
                                                                                                      ); //refs #616
                            buildingComponent->setParticlePtr(new_particle_opposite);
                            new_particle_opposite->addComponent(buildingComponent);
                        }

                        /// Setep 05-bis - Push particle into particle-set and update the particles id counter
                        current_layout_shared.push_back(new_particle_opposite);

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
            /// Old behaviour
            // gps initialization disabled, just generate particles with all values set to zero
            for (int i = 0; i < currentConfiguration.particles_number; i++)
            {
                //Particle zero_part(i, default_mtn_model);
                shared_ptr<Particle> zero_part = make_shared<Particle>(node_handle, &tf_listener, i, default_mtn_model); //refs #616
                MatrixXd tmp_cov = default_mtn_model.getErrorCovariance();
                zero_part->setParticleSigma(tmp_cov);
                current_layout_shared.push_back(zero_part);
            }
        }

        /// Update particle_set size
        LayoutManager::num_particles = currentConfiguration.particles_number + num_particles; // + num_particles because we can have more particles than expected, since the driving direction checks

        // Update first_run flag
        // LayoutManager::layoutManagerFirstRun = false; //moving this to LayoutEstimation Loop

        // print particle poses
        vector<shared_ptr<Particle>>::iterator particle_itr;
        ROS_ERROR_STREAM("NUMERO DI LAYOUT (how many particles): " << current_layout_shared.size());
        for ( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
        {
            ostringstream tmp_convert;   // stream used for the conversion
            tmp_convert << "Particle ID: " << (*particle_itr)->getId();
            (*particle_itr)->getParticleState().printState(tmp_convert.str());

            // Normalize weights
            // refs #445
            (*particle_itr)->setParticleScore((*particle_itr)->getParticleScore() / current_layout_shared.size());
            ROS_DEBUG_STREAM("Updating score of Particle " << (*particle_itr)->getId() << "\t with new value: " << (*particle_itr)->getParticleScore());
        }


        // BUILD POSEARRAY MSG
        // Get particle-set
        geometry_msgs::PoseArray array_msg = LayoutManager::buildPoseArrayMsg(current_layout_shared);
        array_msg.header.stamp = ros::Time::now();
        ROS_INFO_STREAM("Initializing RLE. Particle set size: " << array_msg.poses.size());
        array_msg.header.frame_id = "local_map";
        ROS_INFO_STREAM("All particles state6dof refers to frame id: " << array_msg.header.frame_id.c_str());
        // Publish it (the array)!
        LayoutManager::array_pub.publish(array_msg);

        LayoutManager::publishMarkerArray(1); //normalization factor 1, as all particles score value is 1 or should be ..

    } // end if(first_run)

}// end reconfigure callback


///
/// \brief LayoutManager::publishMarkerArray
/// \param normalizationFactor
///
/// Callback called on nav_msg::Odometry arrival
///
void LayoutManager::publishMarkerArray(double normalizationFactor)
{

    //normalizeParticleSet();

    //the same but with marker-array
    marker_array.markers.clear();
    for (int i = 0; i < current_layout_shared.size(); i++)
    {
        shared_ptr<Particle> p = current_layout_shared.at(i);
        geometry_msgs::Pose pose = p->getParticleState().toGeometryMsgPose();

        tf::Quaternion q;
        tf::quaternionMsgToTF(pose.orientation, q);
        q = q.normalize();
        tf::quaternionTFToMsg(q, pose.orientation);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "local_map";
        marker.header.stamp = ros::Time();
        marker.ns = "particle_set";
        marker.id = p->getId();
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
        marker.color.a = p->getParticleScore() / normalizationFactor;
        marker.color.r = 0;
        marker.color.g = p->getParticleScore() / normalizationFactor;
        marker.color.b = 0;

        marker_array.markers.push_back(marker);

        ROS_DEBUG_STREAM("PUBLISHING INITIAL ARROWS: " << i << " P.score: " << std::setprecision(5) << p->getParticleScore() << " L-inf: " << normalizationFactor << " resulting RGB-alpha " << marker.color.a);

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

void LayoutManager::buildingsCallback(const building_detection::FacadesList &facades)
{
    std::vector<road_layout_estimation::Facade> facades_list;
    facades_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto facade_msg : facades.facades)
    {
        road_layout_estimation::Facade facade(facade_msg);
        facades_list.push_back(facade);
        *facades_cloud_ += *(facade.pcl);

    }

    if (bestParticle)
    {
        Eigen::Affine3d particle_transform = Eigen::Affine3d::Identity();
        particle_transform.translation() << bestParticle->getParticleState().getPosition()[0],  bestParticle->getParticleState().getPosition()[1], 0.;
        particle_transform.rotate(bestParticle->getParticleState().getRotation());
        //tf::transformTFToEigen(GPS_RTK_LOCAL_POSE, particle_transform);
        pcl::transformPointCloud(*facades_cloud_, *facades_cloud_, particle_transform);

        sensor_msgs::PointCloud2 tmp_facades_cloud;
        pcl::toROSMsg(*facades_cloud_, tmp_facades_cloud);
        tmp_facades_cloud.height = facades_cloud_->height;
        tmp_facades_cloud.width = facades_cloud_->width;
        tmp_facades_cloud.header.frame_id = "local_map";
        tmp_facades_cloud.header.stamp = ros::Time::now();
        facades_pub.publish(tmp_facades_cloud);
    }

    // Iterate through all particles
    for ( auto particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
    {
        // Retrieve the RoadLane component from the particle-iterator
        LayoutComponent_Building* BuildingComponentPtr = (*particle_itr)->giveMeThatComponent<LayoutComponent_Building>();
        BuildingComponentPtr->getFacades()->clear();
        for (auto facade : facades_list)
        {
            BuildingComponentPtr->getFacades()->push_back(facade);
        }
    }
    ROS_ERROR_STREAM("< Exiting " << __PRETTY_FUNCTION__);
}


void LayoutManager::odometryCallback(const nav_msgs::Odometry& visualOdometryMsg)
{
    pcl::console::TicToc overallODOMETRYCALLBACK;
    overallODOMETRYCALLBACK.tic();
    ROS_INFO_STREAM("--------------------------------------------------------------------------------");
    ROS_INFO_STREAM("Entering OdomCallBackv2, [odometryMessageCounter: " << odometryMessageCounter++ << "]");
    vector<shared_ptr<Particle>>::iterator particle_itr;

    // Publish GPS init markers
    /// useless since the markers does not expire (should not expire)
    // marker_pub.publish(marker1);
    // marker_pub2.publish(marker2);

    if (LayoutManager::first_msg)
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
    ROS_DEBUG_STREAM("void LayoutManager::odometryCallback2 Delta Timestamp    : " << deltaOdomTime);

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
    ///vector<Particle> particles = current_layout_shared;
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
    Vector3d average_pose_total, average_pose_cluster;
    average_pose_total.setZero();
    average_pose_cluster.setZero();
    //Eigen::Quaterniond average_quaternion = Eigen::Quaterniond::Identity();
    tf::Quaternion average_quaternion = tf::createIdentityQuaternion();
    average_quaternion.setX(0);
    average_quaternion.setY(0);
    average_quaternion.setZ(0);
    average_quaternion.setW(0);

    // CALCULATING TOTAL SCORE FOR NORMALIZATION (used in clustering)
    double tot_score = 0.0f;
    for ( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
    {
        //  ROS_INFO_STREAM("PARTICLE "<< std::setprecision(5) << (*particle_itr).getId() << "\t" << (*particle_itr).getParticleScore() << "\t" << tot_score);
        tot_score += (*particle_itr)->getParticleScore();
    }

    ///////////////////////////////////////////////////////////////////////////
    // CLUSTERING PHASE
    bool enabled_clustering = true;
    int best_cluster = -1;
    int best_cluster_size = 0;
    double cluster_score = 0.0f;
    double best_cluster_score = -1.0f;
    pcl::console::TicToc ttcluster;
    if (enabled_clustering)
    {
        ttcluster.tic();
        ///////////////////////////////////////////////////////////////////////////
        // FIRST STEP, every particle in_cluster value = -1
        for ( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
        {
            (*particle_itr)->in_cluster = -1;
        }
        ///////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////
        // SECOND STEP, clustering. double for.
        int cluster_INDEX = 0;
        double euclidean_distance = 0.0f;
        double angle_distance = 0.0f;
        double euclidean_threshold = 5.00f; //meters
        double angle_threshold     = 0.20f; //radians
        vector<shared_ptr<Particle>>::iterator inner_particle_itr;
        for ( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
        {

            if ((*particle_itr)->in_cluster == -1)
                (*particle_itr)->in_cluster = cluster_INDEX++;

            for ( inner_particle_itr = current_layout_shared.begin(); inner_particle_itr != current_layout_shared.end(); inner_particle_itr++ )
            {
                if (particle_itr == inner_particle_itr)
                    continue;
                if ((*inner_particle_itr)->in_cluster == -1)
                    continue;

                euclidean_distance = sqrt(   (*particle_itr)->getParticleState().getPosition()(0) - (*inner_particle_itr)->getParticleState().getPosition()(0) +
                                             (*particle_itr)->getParticleState().getPosition()(1) - (*inner_particle_itr)->getParticleState().getPosition()(1) +
                                             (*particle_itr)->getParticleState().getPosition()(2) - (*inner_particle_itr)->getParticleState().getPosition()(2)
                                         );

                Eigen::Quaterniond q1 = Eigen::Quaterniond((*particle_itr)->getParticleState().getRotation());
                Eigen::Quaterniond q2 = Eigen::Quaterniond((*inner_particle_itr)->getParticleState().getRotation());
                tf::Quaternion t1, t2;
                t1.setX(q1.x());
                t2.setX(q2.x());
                t1.setY(q1.y());
                t2.setY(q2.y());
                t1.setZ(q1.z());
                t2.setZ(q2.z());
                t1.setW(q1.w());
                t2.setW(q2.w());

                angle_distance = t1.angleShortestPath(t2);

                if (euclidean_distance < euclidean_threshold)
                    if (angle_distance < angle_threshold)
                        (*inner_particle_itr)->in_cluster  = (*particle_itr)->in_cluster;

            }
        }
        ///////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////
        //STEP3 - searching best cluster
        vector<double> clusters;
        clusters.resize(cluster_INDEX);
        for ( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
        {
            clusters[(*particle_itr)->in_cluster] += (*particle_itr)->getParticleScore() / tot_score;
        }

        best_cluster = -1;
        best_cluster_score = -1.0f;
        for (int looking_for_best_cluster = 0; looking_for_best_cluster < cluster_INDEX; looking_for_best_cluster++ )
        {
            if (clusters[looking_for_best_cluster] > best_cluster_score)
            {
                best_cluster_score = clusters[looking_for_best_cluster];
                best_cluster = looking_for_best_cluster;
            }
        }
        cluster_score = 0.0f;
        best_cluster_size = 0;
        for ( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
        {
            if ((*particle_itr)->in_cluster == best_cluster)
            {
                cluster_score += (*particle_itr)->getParticleScore();
                best_cluster_size++;
            }
        }
        cout << "Best cluster size: " << best_cluster_size << endl;
        ROS_ERROR_STREAM("CLUSTERING\t" << ttcluster.toc());
        ///////////////////////////////////////////////////////////////////////////
    }


    ///////////////////////////////////////////////////////////////////////////
    // CREATING STATISTICS FOR RLE OUTPUT

    pcl::console::TicToc ttstat, ttstat1, ttstat2, ttstat3, ttstat4;
    ttstat.tic();
    bool enabled_statistics = true;
    if (enabled_statistics)
    {
        double average_distance_total = 0.0f;
        double average_distance_cluster = 0.0f;
        ttstat1.tic();
        for ( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
        {
            //cout << "particle in_cluster: " << (*particle_itr).in_cluster << " and the best is: " << best_cluster << endl;

            // CHECK IF THE PARTICLE IS IN THE BEST CLUSTER
            //if (enabled_clustering && ((*particle_itr)->in_cluster != best_cluster) )
            //    continue;

            state = (*particle_itr)->getParticleState();
            //if (enabled_clustering)
            if ((*particle_itr)->in_cluster == best_cluster)
                average_pose_cluster += state.getPosition() * (*particle_itr)->getParticleScore() / cluster_score; //tot_score;
            //else
            average_pose_total += state.getPosition() * (*particle_itr)->getParticleScore() / tot_score;

            //sum += (*particle_itr).getParticleScore() / tot_score;
            Eigen::Quaterniond q = Eigen::Quaterniond(state.getRotation());
            tf::Quaternion t;
            t.setX(q.x());
            t.setY(q.y());
            t.setZ(q.z());
            t.setW(q.w());

            //if (enabled_clustering)
//                average_quaternion += t.slerp(tf::createIdentityQuaternion(), (*particle_itr)->getParticleScore() / cluster_score);
            //else
            average_quaternion += t.slerp(tf::createIdentityQuaternion(), (*particle_itr)->getParticleScore() / tot_score);

            //if (enabled_clustering)
            if ((*particle_itr)->in_cluster == best_cluster)
                average_distance_cluster += (*particle_itr)->getDistance_to_closest_segment() * (*particle_itr)->getParticleScore() / cluster_score;
            //else
            average_distance_total += (*particle_itr)->getDistance_to_closest_segment() * (*particle_itr)->getParticleScore() / tot_score;
        }
        average_quaternion.normalize();
        ROS_ERROR_STREAM("AVERAGING\t" << ttstat1.toc());

        //VARIANZA
        double varianza_total = 0.0f;
        double varianza_cluster = 0.0f;
        for ( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
        {
            state = (*particle_itr)->getParticleState();
            double dx;
            double dy;
            state = (*particle_itr)->getParticleState();
            dx = state.getPosition()(0) - average_pose_total(0);
            dy = state.getPosition()(1) - average_pose_total(1);
            varianza_total += sqrt(dx * dx + dy * dy) * (*particle_itr)->getParticleScore() / tot_score;
            if ((*particle_itr)->in_cluster == best_cluster)
            {
                dx = state.getPosition()(0) - average_pose_cluster(0);
                dy = state.getPosition()(1) - average_pose_cluster(1);
                varianza_cluster += sqrt(dx * dx + dy * dy) * (*particle_itr)->getParticleScore() / cluster_score;
            }
        }
        ///////////////////////////////////////////////////////////////////////////


        // PUBLISHING THE AVERAGE POSE (POSITION+ORIENTATION) AS AN ODOMETRY MESSAGE (NAVMSG)
        nav_msgs::Odometry odometry;
        odometry.header.frame_id = "/local_map";
        odometry.header.stamp = ros::Time::now();

        odometry.pose.pose.position.x = average_pose_total(0);
        odometry.pose.pose.position.y = average_pose_total(1);
        odometry.pose.pose.position.z = average_pose_total(2);
        tf::quaternionTFToMsg(average_quaternion, odometry.pose.pose.orientation);
        double roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

        publisher_average_pose.publish(odometry); // Odometry message


        ttstat2.tic();
        ifstream RTK;
        double from_latitude, from_longitude, from_altitude, to_lat, to_lon;
        //TODO: find an alternative to this shit
        int start_frame = visualOdometryMsg.header.seq + 850; // START FRAME AAAAAAAAAAAAAAAAAAA QUI MODIFICA @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 120 o 0 qui zero non uno...
        cout            << "/media/DiscoEsternoGrosso/KITTI_RAW_DATASET/ROAD/2011_10_03_drive_0042_sync/oxts/data/" << boost::str(boost::format("%010d") % start_frame) <<  ".txt" << endl;
        RTK.open(((string)("/media/DiscoEsternoGrosso/KITTI_RAW_DATASET/ROAD/2011_10_03_drive_0042_sync/oxts/data/" + boost::str(boost::format("%010d") % start_frame) + ".txt")).c_str());
        if (!RTK.is_open())
        {
            cout << "ERROR OPENING THE extraordinary kind FILE!" << endl;
            ros::shutdown();
        }
        RTK >> from_latitude >> from_longitude >> from_altitude;
        double roll_gps, pitch_gps, yaw_gps;
        RTK >> roll_gps >> pitch_gps >> yaw_gps;
        cout <<  "LAT LON FROM GPS FILE " << from_latitude << "\t" << from_longitude << endl;
        RTK.close();
        ROS_ERROR_STREAM("RTK READING\t" << ttstat2.toc());

        sensor_msgs::NavSatFix gps_fix;
        gps_fix.header.frame_id = "/map";
        gps_fix.header.stamp = visualOdometryMsg.header.stamp;
        gps_fix.latitude = from_latitude;
        gps_fix.longitude = from_longitude;
        gps_fix.altitude = from_altitude;
        gps_pub.publish(gps_fix);


        /*
        *      SAVING GPS-RTK PART
        */
        // Get XY values from GPS coords
        ira_open_street_map::latlon_2_xyRequest query_latlon2xy;
        query_latlon2xy.latitude = from_latitude;
        query_latlon2xy.longitude = from_longitude;
        //ira_open_street_map::latlon_2_xyResponse response_latlon2xy;
        //if (LayoutManager::latlon_2_xy_client.call(query_latlon2xy, response_latlon2xy))
        tf::Stamped<tf::Pose> RTK_map_frame, RTK_local_map_frame;
        if (true)
        {
            ttstat3.tic();
            double a = from_latitude;
            double b = from_longitude;

            latlon2xy_helper(a, b);
            //ROS_ERROR_STREAM("check this\t" << a << "\t" << response_latlon2xy.x);
            //ROS_ERROR_STREAM("check this\t" << b << "\t" << response_latlon2xy.y);

            //cout << std::setprecision(16) << response_latlon2xy;
            // --- RTK_map_frame.setOrigin(tf::Vector3(response_latlon2xy.x, response_latlon2xy.y, 0));
            RTK_map_frame.setOrigin(tf::Vector3(a, b, 0));
            //RTK_map_frame.setRotation(tf::createIdentityQuaternion());
            RTK_map_frame.setRotation(tf::createQuaternionFromRPY(roll_gps, pitch_gps, yaw_gps));
            RTK_map_frame.frame_id_ = "/map";

            // Transform pose from "map" to "local_map"
            try
            {
                tf_listener.transformPose("local_map", ros::Time(0), RTK_map_frame, "map", RTK_local_map_frame);
                GPS_RTK_LOCAL_POSE = RTK_local_map_frame;
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR_STREAM("LayoutManager.cpp says: %s" << ex.what());
                ROS_ERROR_STREAM("     Transform RTK pose from map to local_map");
                ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
            }

            visualization_msgs::Marker RTK_MARKER;
            RTK_MARKER.header.frame_id = "local_map";
            RTK_MARKER.header.stamp = ros::Time::now();
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


            // PUBLISHING ALMOST THE SAME as POSITION (navmsg) BUT ONLY THE POSE AS A SMALL POINT LIKE THE GPS
            visualization_msgs::Marker LOCALIZATION_MARKER;
            LOCALIZATION_MARKER.header.frame_id = "local_map";
            LOCALIZATION_MARKER.header.stamp = ros::Time::now();
            LOCALIZATION_MARKER.ns = "LOCALIZATION_MARKER";
            LOCALIZATION_MARKER.id = visualOdometryMsg.header.seq; // same as image from kitti dataset
            LOCALIZATION_MARKER.type = visualization_msgs::Marker::CYLINDER;
            LOCALIZATION_MARKER.action = visualization_msgs::Marker::ADD;
            LOCALIZATION_MARKER.pose.orientation.w = 1;
            LOCALIZATION_MARKER.scale.x = 0.5;
            LOCALIZATION_MARKER.scale.y = 0.5;
            LOCALIZATION_MARKER.scale.z = 0.5;
            LOCALIZATION_MARKER.color.a = 1.0;
            LOCALIZATION_MARKER.color.r = 1.0;
            LOCALIZATION_MARKER.color.g = 0.0;
            LOCALIZATION_MARKER.color.b = 1.0;
            LOCALIZATION_MARKER.pose.position.x = average_pose_total(0);
            LOCALIZATION_MARKER.pose.position.y = average_pose_total(1);
            LOCALIZATION_MARKER.pose.position.z = average_pose_total(2);
            marker_array_positions.markers.push_back(LOCALIZATION_MARKER);
            publisher_average_position.publish(marker_array_positions);


            // Push back line_list
            publisher_GT_RTK.publish(marker_array_GT_RTK);

            RTK_GPS_out_file << visualOdometryMsg.header.seq << " " << setprecision(16) <<
                             RTK_local_map_frame.getOrigin().getX() << " " << RTK_local_map_frame.getOrigin().getY() << " " << RTK_local_map_frame.getOrigin().getZ() << " " <<
                             0 << " " << 0 << " " << 0 << " " <<
                             0 << " " << 0 << " " << 0 << " " << 0 << " " <<
                             tot_score / current_layout_shared.size() << " " <<
                             query_latlon2xy.latitude << " " << query_latlon2xy.longitude << "\n";
            RTK_GPS_out_file.flush();


            //        cout  << msg.header.seq << " " << setprecision(16) <<
            //                            RTK_local_map_frame.getOrigin().getX() << " " << RTK_local_map_frame.getOrigin().getY() << " " << RTK_local_map_frame.getOrigin().getZ() << " " <<
            //                            0 << " "<< 0 << " "<< 0 << " " <<
            //                            0 << " " << 0 << " " << 0 << " " << 0 << " " <<
            //                            tot_score / current_layout_shared.size() << " " <<
            //                            query_latlon2xy.latitude << " " << query_latlon2xy.longitude << "\n";

            ROS_ERROR_STREAM("SAVING RTK\t" << ttstat3.toc());
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
        average_pose_local_map_frame.frame_id_ = "local_map";
        average_pose_local_map_frame.setOrigin(tf::Vector3(average_pose_total(0), average_pose_total(1), average_pose_total(2)));
        average_pose_local_map_frame.setRotation(tf::createIdentityQuaternion());
        // Transform pose from "local_map" to "map"
        try
        {
            ttstat4.tic();
            tf_listener.transformPose("map", ros::Time(0), average_pose_local_map_frame, "local_map", average_pose_map_frame);

            //ira_open_street_map::xy_2_latlonRequest query_xy2latlon;
            //ira_open_street_map::xy_2_latlonResponse response_xy2latlon;
            double a = average_pose_map_frame.getOrigin().getX();
            double b = average_pose_map_frame.getOrigin().getY();
            //query_xy2latlon.x = average_pose_map_frame.getOrigin().getX();
            //query_xy2latlon.y = average_pose_map_frame.getOrigin().getY();
            latlon2xy_helper(a, b);
            to_lat = a;
            to_lon = b;
            //if (LayoutManager::xy_2_latlon_client.call(query_xy2latlon, response_xy2latlon))
            //{
            //    // to_lat && to_lon are then the average values (of all particles) in LAT/LON UTM
            //    to_lat = response_xy2latlon.latitude;
            //    to_lon = response_xy2latlon.longitude;
            //}
            //else
            //{
            //    ROS_ERROR_STREAM("   Failed to call 'xy_2_latlon_2_srv' service");
            //    ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
            //    return;
            //}

            // -------------------------------------------------------------------------------------------------------------------------------------
            // SAVE RESULTS TO OUTPUT FILE:
            tf::Matrix3x3(average_quaternion).getRPY(roll, pitch, yaw);
            double dx = (RTK_local_map_frame.getOrigin().getX() - average_pose_total(0));
            double dy = (RTK_local_map_frame.getOrigin().getY() - average_pose_total(1));
            double distance1 = sqrt(dx * dx + dy * dy);

            dx = (RTK_local_map_frame.getOrigin().getX() - average_pose_cluster(0));
            dy = (RTK_local_map_frame.getOrigin().getY() - average_pose_cluster(1));
            double distance2 = sqrt(dx * dx + dy * dy);
            /*RLE_out_file << visualOdometryMsg.header.seq << " " << setprecision(16) <<
                         average_pose_total(0) << " " << average_pose_total(1) << " " << average_pose_total(2) << " " <<
                         roll << " " << pitch << " " << yaw << " " <<
                         average_quaternion.getX() << " " << average_quaternion.getY() << " " << average_quaternion.getZ() << " " << average_quaternion.getW() << " " <<
                         tot_score / current_layout_shared.size() << " " <<
                         to_lat << " " << to_lon << " " <<
                         average_distance_total << "\n";*/
            RLE_out_file << visualOdometryMsg.header.seq << ";" << setprecision(16) <<
                         average_distance_total << ";" <<
                         average_distance_cluster << ";" <<
                         distance1 << ";" <<
                         distance2 << ";" <<
                         varianza_total << ";" <<
                         varianza_cluster << ";" << "\n";
            RLE_out_file.flush();


            //cout << start_frame << " " << setprecision(16) <<
            //     average_pose(0) << " " << average_pose(1) << " " << average_pose(2) << " " <<
            //     roll << " " << pitch << " " << yaw << " " <<
            //     average_quaternion.getX() << " " << average_quaternion.getY() << " " << average_quaternion.getZ() << " " << average_quaternion.getW() << " " <<
            //     tot_score / current_layout_shared.size() << " " <<
            //     to_lat << " " << to_lon << " " <<
            //     average_distance << "\n";

            ROS_ERROR_STREAM("SAVING RLE\t" << ttstat4.toc());

        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR_STREAM("LayoutManager.cpp says: %s" << ex.what());
            ROS_ERROR_STREAM("     Transform AVERAGE pose from local_map to map");
            ros::shutdown();
        }


        /*
        *      SAVING LIBVISO PART
        */

        // TODO: calculate LAT LON
        //        tf::StampedTransform VO;
        //        try
        //        {
        //            tf_listener.lookupTransform("/local_map", "/visual_odometry_car_frame", ros::Time(0), VO);
        //
        //            tf::Matrix3x3(VO.getRotation()).getRPY(roll, pitch, yaw);
        //
        //            LIBVISO_out_file << visualOdometryMsg.header.seq << " " << setprecision(16) <<
        //                             VO.getOrigin().getX()  << " " << VO.getOrigin().getY()  << " " << VO.getOrigin().getZ()  << " " <<
        //                             roll << " " << pitch << " " << yaw << " " <<
        //                             VO.getRotation().getX() << " " << VO.getRotation().getY() << " " << VO.getRotation().getZ() << " " << VO.getRotation().getW() << " " <<
        //                             tot_score / current_layout_shared.size() << "\n";
        //            LIBVISO_out_file.flush();
        //        }
        //        catch (tf::TransformException &ex)
        //        {
        //            ROS_WARN_STREAM("VO");
        //        }
        ROS_ERROR_STREAM("STATISTICS: " << ttstat.toc());
    }
    // -------------------------------------------------------------------------------------------------------------------------------------


    ROS_DEBUG_STREAM("Exiting OdomCallBack v2\t overall time: \t" << overallODOMETRYCALLBACK.toc());

    //Start the timer.  Does nothing if the timer is already started.
    this->rleStart();
}


///
/// \brief LayoutManager::checkHasMoved
/// \return true if car has moved beyond the the threshold matrix
///
/// Check if car has moved or not by confronting odometry matrix and motion threshold matrix
///
bool LayoutManager::checkHasMoved()
{
    //MatrixXd diff_matrix = (visual_odometry.getOdometry().array() > motion_threshold).cast<double>();
    //  MatrixXd diff_matrix = MatrixXd::Ones(12,12);
    //  return diff_matrix.count() > 0; //every position over the threshold will count as 1

    return true;
}

/**
 * @brief LayoutManager::roadLaneCallback
 * @param msg_lines
 *
 * This is the callback to the ISIS LAB line detector interface.
 * Calling 'filter' inside RoadLaneComponent to execute the analysis HMM/DBN
 *
 * This function does not delete the component RoadLane, rather updates it.
 *
 */
void LayoutManager::roadLaneCallback(const road_layout_estimation::msg_lines &msg_lines)
{
    ROS_ERROR_STREAM("> Entering " << __PRETTY_FUNCTION__);

    // Variables declaration
    vector<shared_ptr<Particle>>::iterator particle_itr;

    // Iterate through all particles
    for ( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
    {
        // Retrieve the RoadLane component from the particle-iterator
        LayoutComponent_RoadLane* RoadLaneComponentPtr = (*particle_itr)->giveMeThatComponent<LayoutComponent_RoadLane>();

        // check if we have the pointer (null otherwise)
        if (RoadLaneComponentPtr)
            RoadLaneComponentPtr->filter(msg_lines); //call the filter, HMM/DBN. refs #519
        else
            ROS_WARN("No RoadLaneComponent found");
    }

    ROS_ERROR_STREAM("< Exiting " << __PRETTY_FUNCTION__);
}

/**
 * @brief LayoutManager::roadStateCallback
 * @param msg_lines
 * @todo {Parametrize snapParticle.request.max_distance_radius value}
 *
 * This function update (by creating new ones, means deleting + creating) the
 * RoadState Component. It calls the snapParticle Service to get the wayId,
 * but IF NOT found, the component is NOT created again.
 *
 * This code is related to the following REDMINE issues: #520
 *
 */
void LayoutManager::roadStateCallback(const road_layout_estimation::msg_lines &msg_lines)
{

    ROS_WARN_STREAM("> Entering roadStateCallback");

    ira_open_street_map::snap_particle_xy   snapParticleRequestResponse; ///< ROS parameter for service call
    //ira_open_street_map::oneway             oneWayRequestResponse;       ///< ROS parameter for service call

    road_layout_estimation::msg_lines modified_msg_lines;
    modified_msg_lines = msg_lines;
    vector<shared_ptr<Particle>>::iterator particle_itr;
    ROS_WARN_STREAM("DETECTOR WIDTH: " << msg_lines.width << ", NAIVE: " << msg_lines.naive_width);

    for ( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
    {
        tf::Stamped<tf::Pose> tf_pose_map_frame = toGlobalFrame((*particle_itr)->getParticleState().getPosition());
        snapParticleRequestResponse.request.x = tf_pose_map_frame.getOrigin().getX();
        snapParticleRequestResponse.request.y = tf_pose_map_frame.getOrigin().getY();
        snapParticleRequestResponse.request.max_distance_radius = 20;  // TODO: parametrize this value
        if (snap_particle_xy_client.call(snapParticleRequestResponse))
        {
            //ROS_WARN_STREAM("ID: "<<snapParticleRequestResponse.response.way_id);
            modified_msg_lines.way_id = snapParticleRequestResponse.response.way_id;
        }

        LayoutComponent_RoadState* RoadStateComponentPtr = (*particle_itr)->giveMeThatComponent<LayoutComponent_RoadState>();
        RoadStateComponentPtr->setMsg_lines(modified_msg_lines);

    }

    ROS_INFO_STREAM("< Exiting roadStateCallback");
}



/**
 * @brief LayoutManager::componentsEstimation
 * Estimate particles' components using particle filter
 */
void LayoutManager::componentsEstimation()
{
    pcl::console::TicToc ttsampling;
    pcl::console::TicToc ttperturb;
    pcl::console::TicToc ttweight;
    double ttt = 0;

    ROS_DEBUG("> Entering componentsEstimation");

    // STEP 1: SAMPLING (PREDICT COMPONENTS POSES)
    sampling();

    // STEP 2: PERTURBATE COMPONENT POSES
    //componentsPerturbation();

    // STEP 3: WEIGHT LAYOUT-COMPONENTS
    calculateLayoutComponentsWeight();

    ROS_DEBUG("< Exiting componentsEstimation");
}

///
/// \brief LayoutManager::sampling
///
/// PARTICLE FILTER, STEP 1:
/// Cicle through all the particles, and call their function "propagate-components"
///
void LayoutManager::sampling()
{
    ROS_DEBUG_STREAM("> Entering Sampling of all components");
    vector<shared_ptr<Particle>>::iterator itr;
    int partinumber = 0;
    for ( itr = current_layout_shared.begin(); itr != current_layout_shared.end(); itr++ )
    {
        ROS_INFO_STREAM_ONCE("--- Propagating components of particle, ID: " << (*itr)->getId() << " ---");

        // QtCreator annoying bug... solved with this 'trick'
        shared_ptr<Particle> particle = *itr;
        particle->propagateLayoutComponents(partinumber++);

        //itr->propagateLayoutComponents();
    }
    ROS_DEBUG_STREAM("< Exiting Sampling of all components");
}

///
/// \brief LayoutManager::componentsPerturbation
///
/// PARTICLE FILTER, STEP 2:
///
void LayoutManager::componentsPerturbation()
{

    // first, iterate over all particles of 'current_layout_shared'
    for (int i = 0; i < current_layout_shared.size(); i++)
    {
        shared_ptr<Particle> p = current_layout_shared.at(i);
        vector<LayoutComponent*> vec = p->getLayoutComponents();

        // second, iterate over all layout-components of current 'particle'
        for (int j = 0; j < vec.size(); j++)
        {
            LayoutComponent* lc = vec.at(j);
            lc->componentPerturbation();
        }
    }
}

///
/// \brief LayoutManager::calculateLayoutComponentsWeight
///
/// PARTICLE FILTER, STEP 3:
/// andiamo a vedere quanto bene fitta la componente della singola particella nella realtà
///
void LayoutManager::calculateLayoutComponentsWeight()
{
    tt.tic();
    ROS_DEBUG_STREAM("> Entering calculateLayoutComponentsWeight (and then calling *virtual* calculateComponentScore ");
    // first, iterate over all particles of 'current_layout_shared'
    for (int i = 0; i < current_layout_shared.size(); i++)
    {
        shared_ptr<Particle> p = current_layout_shared.at(i);
        ROS_DEBUG_STREAM("Particle p.getId() : " << p->getId());
        vector<LayoutComponent*> vec = p->getLayoutComponents();
        int sss = vec.size();
        // second, iterate over all layout-components of current 'particle'
        for (int j = 0; j < vec.size(); j++)
        {
            LayoutComponent* lc = vec.at(j);
            lc->calculateComponentScore();
        }
    }
    ROS_DEBUG_STREAM("< Exiting calculateLayoutComponentsWeight (and then calling *virtual* calculateComponentScore ");

    double ttt = tt.toc();
    if (ttt > 1)
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ttt << " milliseconds");
}
/** **************************************************************************************************************/


/**
 * @brief LayoutManager::calculateGeometricScores
 * @param particle_itr
 *
 * This is the geometric part, EUCLIDEAN and ANGULAR distance from the OSM road net
 * As written in #522, this should become part of a new OSM-DISTANCE component.
 * These two valules are stored in the particle parameters:
 *          pose_diff_score_component
 *          final_angle_diff_score_component
 */
void LayoutManager::calculateGeometricScores(const shared_ptr<Particle>& particle_itr)//(Particle *particle_itr)
{
    // SCORE using OpenStreetMap-----------------------------------------------------------------------------
    // update particle score using OpenStreetMap
    if (LayoutManager::openstreetmap_enabled)
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
        Vector3d particle_position_state = (*particle_itr).getParticleState().getPosition();
        geometry_msgs::PoseStamped pose_local_map_frame;
        pose_local_map_frame.header.frame_id = "local_map";
        pose_local_map_frame.header.stamp = ros::Time::now();
        pose_local_map_frame.pose.position.x = particle_position_state(0);
        pose_local_map_frame.pose.position.y =  particle_position_state(1);
        pose_local_map_frame.pose.position.z =  particle_position_state(2);
        Eigen::Quaterniond particle_orientation_state_quaterion((*particle_itr).getParticleState().getRotation());
        pose_local_map_frame.pose.orientation.w = particle_orientation_state_quaterion.w();
        pose_local_map_frame.pose.orientation.x = particle_orientation_state_quaterion.x();
        pose_local_map_frame.pose.orientation.y = particle_orientation_state_quaterion.y();
        pose_local_map_frame.pose.orientation.z = particle_orientation_state_quaterion.z();

        tf::Stamped<tf::Pose> tf_pose_map_frame, tf_pose_local_map_frame;
        tf::poseStampedMsgToTF(pose_local_map_frame, tf_pose_local_map_frame);
        // Transform pose from "local_map" to "map"
        try
        {
            tf_listener.transformPose("map", ros::Time(0), tf_pose_local_map_frame, "local_map", tf_pose_map_frame);

        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR_STREAM("%s" << ex.what());
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
        snapParticle_serviceMessage.request.max_distance_radius = 100; // distance radius for finding the closest nodes for particle snap WARNING: parametrize this value

        ///////     // THIS IS RELATED WITH #502
        ///////     ira_open_street_map::getDistanceFromLaneCenter getDistanceFromLaneCenter_serviceMessage;
        ///////     getDistanceFromLaneCenter_serviceMessage.request.x = tf_pose_map_frame.getOrigin().getX();
        ///////     getDistanceFromLaneCenter_serviceMessage.request.y = tf_pose_map_frame.getOrigin().getY();
        ///////     //getDistanceFromLaneCenter_serviceMessage.request.way_id = (dynamic_cast<LayoutComponent_RoadState *>((*particle_itr).getLayoutComponents().at(0)))->getWay_id(); //FIX: find the right component (check also the cast)
        ///////     getDistanceFromLaneCenter_serviceMessage.request.way_id = (*particle_itr).getWayIDHelper(); // refs #502
        ///////     getDistanceFromLaneCenter_serviceMessage.request.current_lane = 0; //FIX: unused?!
        ///////
        ///////     if (LayoutManager::getDistanceFromLaneCenter_client.call(getDistanceFromLaneCenter_serviceMessage))
        ///////     {
        ///////         //ROS_ERROR_STREAM ("STICA!   " << getDistanceFromLaneCenter_serviceMessage.request.way_id << "\t " << getDistanceFromLaneCenter_serviceMessage.response.distance_from_lane_center << "\t" << getDistanceFromLaneCenter_serviceMessage.response.distance_from_way_center);
        ///////     }

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

            ///Augusto: TEST 2. CHECKED LATER IN THE CODE, BUT KNOWN BY ME.
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
            try
            {
                tf_listener.waitForTransform("local_map", "map", ros::Time(0), ros::Duration(1));
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
            double distance = sqrt(dx * dx + dy * dy + dz * dz);

            /// update the particle variable; this is also used in the
            /// CREATING STATISTICS FOR RLE OUTPUT section
            //#522            (*particle_itr).distance_to_closest_segment = distance;

            /// For debuggin purposes: add line to marker array distances
            //publishMarkerArrayDistances((*particle_itr).getId(),
            //                            tf_pose_local_map_frame.getOrigin().getX(),
            //                            tf_pose_local_map_frame.getOrigin().getY(),
            //                            tf_snapped_local_map_frame.getOrigin().getX(),
            //                            tf_snapped_local_map_frame.getOrigin().getY(),
            //                            tf_pose_local_map_frame.getOrigin().getZ());

            //      get PDF score FOR DISTANCE
            //#522            boost::math::normal normal_dist(0, street_distribution_sigma);
            //double pose_diff_score_component = pdf(normal_dist, distance);
            //#522            (*particle_itr).pose_diff_score_component = pdf(normal_dist, distance) / pdf(normal_dist , 0.0f);
            //tf_snapped_local_map_frame.getRotation().getAngleShortestPath(tf_pose_local_map_frame.getRotation()) // check this out, maybe works .. this line isn't tested yet

            // calculate angle difference ---------------------------------------------------------------------------------------
            first_quaternion_diff = tf_snapped_local_map_frame.getRotation().inverse() * tf_pose_local_map_frame.getRotation();
            street_direction = first_quaternion_diff;

            //      get PDF score from first angle
            //#522            boost::math::normal angle_normal_dist(0, angle_distribution_sigma);
            double first_angle_difference = Utils::normalize_angle(first_quaternion_diff.getAngle());
            //#522            double first_angle_diff_score = pdf(angle_normal_dist, first_angle_difference) / pdf(angle_normal_dist, 0.0f);

            //#522            (*particle_itr).final_angle_diff_score_component  = 0.0f;
            double second_angle_diff_score = 0.0f;
            double second_angle_difference = 0.0f;

            //      if street have 2 directions check angle diff with opposite angle
            if (snapParticle_serviceMessage.response.way_dir_opposite_particles)
            {
                tf_snapped_local_map_frame_opposite_direction = tf_snapped_local_map_frame; // COPY TRANSFORM (I PRAY FOR THIS)
                tf_snapped_local_map_frame_opposite_direction.setRotation(tf_snapped_local_map_frame * tf::createQuaternionFromYaw(M_PI)); // INVERT DIRECTION
                second_quaternion_diff = tf_snapped_local_map_frame_opposite_direction.getRotation().inverse() * tf_pose_local_map_frame.getRotation();

                //      get PDF score
                second_angle_difference = Utils::normalize_angle(second_quaternion_diff.getAngle());
                //#522                second_angle_diff_score = pdf(angle_normal_dist, second_angle_difference) / pdf(angle_normal_dist, 0.0f); //TODO: refs #442 check if this normalization is correct or I used the opposite angle

                //#522                //      set score
                //#522                if (second_angle_diff_score > first_angle_diff_score)
                //#522                    (*particle_itr).final_angle_diff_score_component = second_angle_diff_score;
                //#522                else
                //#522                    (*particle_itr).final_angle_diff_score_component = first_angle_diff_score;

                ROS_DEBUG_STREAM("ONEWAY-No \tSPATIAL_DISTANCE: " << distance << "\tANGLE_1: " << first_angle_difference << "\tANGLE_2: " << second_angle_difference);
            }
            else
            {
                ROS_DEBUG_STREAM("ONEWAY-Yes\tSPATIAL_DISTANCE: " << distance << "\tANGLE_1: " << first_angle_difference);
                //#522                (*particle_itr).final_angle_diff_score_component = first_angle_diff_score;
            }

            ///Augusto: TEST FINAL. ** BE CAREFUL, THIS IS EQUAL TO INVERTED DIRECTION IF TEST2 IS ENABLED .
            transform.setOrigin( tf_pose_local_map_frame.getOrigin());
            transform.setRotation(tf_pose_local_map_frame.getRotation()*street_direction.inverse());
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_map", "tf_pose_local_map_frame_ROTATED"));

            // set particle score
            //(*particle_itr).setParticleScore(street_distribution_alpha/tot_weight * pose_diff_score_component + angle_distribution_alpha/tot_weight * angle_diff_score_component);
            //(*particle_itr).setParticleScore(street_distribution_alpha * pose_diff_score_component * angle_distribution_alpha * final_angle_diff_score);

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
            //#522                (*particle_itr).distance_to_closest_segment = std::numeric_limits<double>::infinity();
            ROS_ERROR_STREAM("RLE Main loop, snap_particle_xy service call");
            ROS_ERROR_STREAM("Either service is down or particle is too far from a street. Shutdown in LayoutManager.cpp");
            ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
        }// end snap particle client
    } // end scoring function with openstreetmap enabled
}



///
/// \brief LayoutManager::calculateScore
/// \param particle_itr
///
/// FORMULA CALCOLO SCORE
///
/// Cardinalità unaria
///  NO: 1- Kalman gain sulla pose della particella
///  2- Somma dei WEIGHT delle varie componenti della particella (ottenuti dal filtraggio a particelle)
///
/// Cardinalità >= 2
///  1- plausibilità di esistenza tra le varie componenti di stesso tipo (due building sovrapposti ecc.) nella stessa particella
///  2- plausibilità di esistenza tra componenti di diverso tipo (building sovrapposto a una macchina ecc.) nella stessa particella
///
/// Nessuna particella verrà eliminata durante il procedimento di calcolo dello score,
/// essa sarà mantenuta in vita nonostante lo score sia basso.
/// In questo modo si evita la possibilità di eliminare dal particle-set ipotesi plausibili che abbiano ricevuto
/// uno score di valore basso per motivi di natura diversa.
///
//void LayoutManager::calculateScore(Particle *particle_itr) //const reference
void LayoutManager::calculateScore(const shared_ptr<Particle>& particle_itr_shared)
{
    ROS_DEBUG_STREAM("Entering calculateScore() -- particleId " << particle_itr_shared->getId());

    //(*particle_itr).setParticleScore(street_distribution_alpha * (*particle_itr).pose_diff_score_component * angle_distribution_alpha * (*particle_itr).final_angle_diff_score);


    vector<LayoutComponent*> layoutComponentVector = (*particle_itr_shared).getLayoutComponents();  //return the pointer to the LayoutComponent Array (vector)
    //LayoutComponent* lc = layoutComponentVector.at(0); //JUST BECAUSE NOW WE HAVE ONLY ONE COMPONENT

    ROS_DEBUG_STREAM("PARTICLE SCORE PRE UPDATE\t" << std::fixed <<  (*particle_itr_shared).getParticleScore());

    // #522    ROS_ASSERT((*particle_itr).pose_diff_score_component        > 0.0f);
    // #522    ROS_ASSERT((*particle_itr).final_angle_diff_score_component > 0.0f);

    /// First sum the first parts coded inside the particle (not the component).
    /// Before the N-number of components here we hace a single layout component
    /// and a getComponentWeight, now it is splitted with this lines and the
    /// following for
    double newScore = 0.0f;
    // #522    double newScore = (
    // #522                          (
    // #522                              street_distribution_alpha    * (*particle_itr).pose_diff_score_component          +
    // #522                              angle_distribution_alpha     * (*particle_itr).final_angle_diff_score_component
    // #522                              //roadState_distribution_alpha * lc->getComponentWeight()
    // #522                          )
    // #522                      );


    /*  This is the second part I was writing about before.
     *  This Iterates through all Layout Components to get the overall score.
     *
     *  Before #522 and #534 here I had to check for the object type to select
     *  the right weight factor, now the weights are already weighted inside the
     *  component.
     */
    double sumOfAlphas = 0.0f;
    for (int j = 0; j < layoutComponentVector.size(); j++)
    {
        LayoutComponent* lc = layoutComponentVector.at(j); //iterator through the Layout Components

        // No need to check what kind of component I have in the pointer! :-)
        newScore += lc->getComponentWeight();

        // The layout component uses all the alphas he need, if more than one the
        // getAlphas virtual routine provides to summarize them
        sumOfAlphas += lc->getAlphas();

        if (dynamic_cast<LayoutComponent_OSMDistance* >(lc))
            ROS_DEBUG_STREAM("COMPONENT OSM_DISTANCE Score = " << lc->getComponentWeight() << "\tSum of Alphas: " << lc->getAlphas());

        if (dynamic_cast<LayoutComponent_Building* >(lc))
            ROS_DEBUG_STREAM("COMPONENT BUILDING     Score = " << lc->getComponentWeight() << "\tSum of Alphas: " << lc->getAlphas());

        //if (dynamic_cast<LayoutComponent_RoadState* >(lc))
        //if (dynamic_cast<LayoutComponent_RoadLane* >(lc))

    }

    // L1 Normalization factor, check if the single values are > 0 (L1, sum)
    // #522    ROS_ASSERT(street_distribution_alpha   > 0.0f);
    // #522    ROS_ASSERT(angle_distribution_alpha    > 0.0f);
    // #522    ROS_ASSERT(roadState_distribution_alpha> 0.0f);
    // #522        double sumOfAlphas = street_distribution_alpha + angle_distribution_alpha + roadState_distribution_alpha;

    ROS_DEBUG_STREAM("Total components weight: " << newScore << "\tSum of Alphas: " << sumOfAlphas << "\tNew 'sumofweights': " << newScore / sumOfAlphas);

    /// Execute normalization
    newScore = newScore / sumOfAlphas;

    /// Set the score to the particle
    (*particle_itr_shared).setParticleScore( (*particle_itr_shared).getParticleScore() * newScore );

    // #522    ROS_DEBUG_STREAM("PARTICLE SCORE DIST: \t" << std::fixed <<  (*particle_itr).pose_diff_score_component             << "\texp(log) : " << exp(log(abs((*particle_itr).pose_diff_score_component)))) ;
    // #522    ROS_DEBUG_STREAM("PARTICLE SCORE ANGL: \t" << std::fixed <<  (*particle_itr).final_angle_diff_score_component      << "\texp(log) : " << exp(log(abs((*particle_itr).final_angle_diff_score_component   )))) ;
    // #522    //ROS_DEBUG_STREAM("COMPONENT SCORE:     \t" << std::fixed <<  lc->getComponentWeight()                              << "\texp(log) : " << exp(log(abs(lc->getComponentWeight())))                 ) ;
    // #522    ROS_DEBUG_STREAM("ALL SCORES SUM :     \t" << std::fixed <<  newScore                                              << "\tmax score: " << sumOfAlphas) ;
    // #522    ROS_DEBUG_STREAM("PARTICLE SCORE TOTAL \t" << std::fixed <<  (*particle_itr).getParticleScore());

    ROS_DEBUG_STREAM("PARTICLE SCORE POST UPDATE\t" << std::fixed <<  (*particle_itr_shared).getParticleScore());
    ROS_DEBUG_STREAM("Exiting calculateScore()");

    /// OLD DARIO IMPLEMENTATION
    /*vector<Particle>::iterator p_itr
    for( p_itr = current_layout_shared.begin(); p_itr != current_layout_shared.end(); p_itr++ )
    {

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
    }*/
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

    vector<shared_ptr<Particle>>::iterator particle_itr;
    std::vector<double> latitudes;
    std::vector<double> longitudes;
    std::vector<double> particleScores;
    std::vector<long int> way_ids;
    unsigned int particle_counter = 0;

    for ( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
    {
        vector<LayoutComponent*> componentVector = (*particle_itr)->getLayoutComponents();
        int components = componentVector.size();

        ROS_DEBUG_STREAM("PARTICLE " << particle_counter++ << "\tComponents Vector Size: " << components);

        for (int i = 0; i < components; i++)
        {
            LayoutComponent *component = componentVector.at(i);
            if (LayoutComponent_RoadState* roadState = dynamic_cast<LayoutComponent_RoadState* >(component))
            {
                ROS_DEBUG_STREAM("ParticleId: " << (*particle_itr)->getId() << "\twayId: " << roadState->getWay_id());
                way_ids.push_back(roadState->getWay_id());
                particleScores.push_back((*particle_itr)->getParticleScore());

                // TRANSFORM AVERAGE POSE TO LAT/LON (NEED CONVERSION FROM LOCAL_MAP TO MAP AND ROS-SERVICE CALL)
                tf::Stamped<tf::Pose> particle_map_frame, particle_local_frame;
                particle_local_frame.frame_id_ = "local_map";
                particle_local_frame.setOrigin(tf::Vector3((*particle_itr)->getParticleState().getPosition()(0), (*particle_itr)->getParticleState().getPosition()(1), (*particle_itr)->getParticleState().getPosition()(2)));
                particle_local_frame.setRotation(tf::createIdentityQuaternion());
                // Transform pose from "local_map" to "map"
                try
                {
                    tf_listener.transformPose("map", ros::Time(0), particle_local_frame, "local_map", particle_map_frame);
                    ira_open_street_map::xy_2_latlon xy2latlon;
                    xy2latlon.request.x = particle_map_frame.getOrigin().getX();
                    xy2latlon.request.y = particle_map_frame.getOrigin().getY();
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
                    ROS_ERROR_STREAM("getAllParticlesLatLonService failed to trasform needed poses %s" << ex.what());
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
    resp.particleNumber = current_layout_shared.size();
    resp.latitudes = latitudes;
    resp.longitudes = longitudes;
    resp.way_ids = way_ids;
    resp.particleScores = particleScores;

    ROS_DEBUG_STREAM("> Exiting getAllParticlesLatLonService");
    return true;
}

/**
 * @brief LayoutManager::layoutEstimation
 * @param timerEvent
 *
 * This routine is called scheduled using the ROS timer
 *
 */
void LayoutManager::layoutEstimation(const ros::TimerEvent& timerEvent)
{
    tt2.tic();

    setCurrent_layoutScore(0.0f);

    ROS_INFO_STREAM("--------------------------------------------------------------------------------");
    this->deltaTimerTime = (timerEvent.current_real - timerEvent.last_real).toSec();

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
            this->deltaTimerTime = this->measurement_model->getDelta_time().toSec();
            ROS_WARN_STREAM("FIRST run! Using measurementModel delta_time for this iteration only: " << this->deltaTimerTime);

            vector<shared_ptr<Particle>>::iterator particle_itr;
            for ( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
            {
                ROS_DEBUG_STREAM("FIRST run! Setting speeds of particle n# " << (*particle_itr)->getId());
                //(*particle_itr).setParticleVelocities(measurement_model->getMeasureDeltaState());             //3D Initialization
                (*particle_itr)->setParticleVelocities(measurement_model->getOrthogonalMeasureDeltaState());     //do not use 3D rotations for initialization
            }

            this->layoutManagerFirstRun = false;
        }
    }

    if (this->deltaTimerTime > 10)
    {
        ROS_WARN_STREAM("Warning, deltaTimer>10 seconds " << this->deltaTimerTime );
        //this->deltaTimerTime = 10;                                      //refs #613  qui originariamente c'era 0.1 al posto di 10
    }                                                                   //refs #613
    else                                                                //refs #613
    {
        //refs #613 questa parte l'ho aggiunta risolvendo bug
        ROS_DEBUG_STREAM("Delta 'corretti'\t" << this->deltaTimerTime);    //refs #613 questa parte l'ho aggiunta risolvendo bug
    }                                                                     //refs #613 questa parte l'ho aggiunta risolvendo bug

    if (this->deltaTimerTime < 0)
    {
        ROS_ERROR_STREAM("deltaTimerTime can not be less than zero. Shutting down everything");
        ros::shutdown();
    }

    // Check if car has moved, if it has moved then estimate new layout.
    // Actually set to TRUE hard coded.
    if ( checkHasMoved() )
    {

        //  ----------------- predict and update layout poses using E.K.F --------------------
        //    This involves *ONLY* the particle EKF routine, none of the components is used!
        //  ----------------------------------------------------------------------------------
        vector<shared_ptr<Particle>>::iterator particle_itr; //this iterator is used in all cycles
        for ( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
        {
            ROS_DEBUG_STREAM("Estimating pose of particle n# " << (*particle_itr)->getId());
            (*particle_itr)->particlePoseEstimation(measurement_model, this->deltaTimerTime, this->deltaOdomTime); //ekf
        }



        // -------------- Sampling + Perturbation + Weight layout-components ------------- //
        // componentsEstimation throws the execution chain over *ALL* particle COMPONENTS
        //     1.sampling --> propagateLayoutComponents --> ComponentPoseEstimation
        //     2.componentsPerturbation();
        //     3.calculateLayoutComponentsWeight();
        this->componentsEstimation(); //the cycle for EVERY particle is inside each call
        // ------------------------------------------------------------------------------- //



        /// ------------------------------ calculate score -------------------------------- ///
        ///
        /// 1. Geometric Values     calculateGeometricScores
        /// 2. Other Scores         calculateScore
        ///
        ///


        /// GET SOME GEOMETRIC VALUES -- check #522 for some ideas about the following FOR cycle
        // #522        for ( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
        // #522            this->calculateGeometricScores(*particle_itr);



        /// COMPUTE NORMALIZATON (disabled, I just print some values here)
        // #522        double minDistance = std::numeric_limits<double>::max();
        // #522        double minAngle    = std::numeric_limits<double>::max();
        // #522        double maxDistance = std::numeric_limits<double>::min();
        // #522        double maxAngle    = std::numeric_limits<double>::min();
        // #522        for ( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
        // #522        {
        // #522            ROS_INFO_STREAM("#distance: "       << (*particle_itr)->pose_diff_score_component <<
        // #522                            "  \tangle (rad): " << (*particle_itr)->final_angle_diff_score_component <<
        // #522                            "  \tcomponent(0): " << (*particle_itr)->getLayoutComponents().at(0)->getComponentWeight()
        // #522                           );
        // #522
        // #522            if ((*particle_itr)->pose_diff_score_component > maxDistance)
        // #522                maxDistance = (*particle_itr)->pose_diff_score_component ;
        // #522            if ((*particle_itr)->pose_diff_score_component < minDistance)
        // #522                minDistance = (*particle_itr)->pose_diff_score_component ;
        // #522            if ((*particle_itr)->final_angle_diff_score_component > maxAngle)
        // #522                maxAngle = (*particle_itr)->final_angle_diff_score_component;
        // #522            if ((*particle_itr)->final_angle_diff_score_component < minAngle)
        // #522                minAngle = (*particle_itr)->final_angle_diff_score_component;
        // #522        }
        // #522        ROS_DEBUG_STREAM("**********************************************************************************");
        // #522        ROS_DEBUG_STREAM("minDistance: " << minDistance << "\tmaxDistance: " << maxDistance);
        // #522        ROS_DEBUG_STREAM("minAngle   : " << minAngle    << "\tmaxAngle   : " << maxAngle);
        // #522        ROS_DEBUG_STREAM("**********************************************************************************");
        //for( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
        //{
        //    ROS_INFO_STREAM("Before normalization\t" << (*particle_itr).pose_diff_score_component << " \tAngle: " << (*particle_itr).final_angle_diff_score_component);
        //    (*particle_itr).pose_diff_score_component = ((*particle_itr).pose_diff_score_component - minDistance )/(maxDistance-minDistance);
        //    (*particle_itr).final_angle_diff_score_component    = ((*particle_itr).final_angle_diff_score_component    - minAngle    )/(maxAngle   -minAngle   );
        //    ROS_INFO_STREAM("Normalized distance \t" << (*particle_itr).pose_diff_score_component << " \tAngle: " << (*particle_itr).final_angle_diff_score_component << endl);
        //}





        /// EVALUATE PARTICLE SCORES AND SAVE BEST PARTICLE POINTER
        //Particle *bestParticle   = NULL;
        double bestParticleScore = 0.0f;
        double currentParticleScore      = 0.0f;
        for ( particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
        {
            this->calculateScore(*particle_itr);             /// Evaluate the SCORE of the particle

            currentParticleScore = (*particle_itr)->getParticleScore();  /// Get the score of the current particle

            current_layoutScore += currentParticleScore;

            if (currentParticleScore > bestParticleScore)
            {
                bestParticleScore = (*particle_itr)->getParticleScore();
                bestParticle = *(particle_itr);
            }
        }


        /// Publish debug information
        // #522        try
        // #522        {
        // #522            if (bestParticle != NULL)
        // #522            {
        // #522                road_layout_estimation::msg_debugInformation debugInfoMessage;
        // #522
        // #522                debugInfoMessage.distance_Euclidean    = (*bestParticle).pose_diff_score_component;
        // #522                debugInfoMessage.distance_Angular      = (*bestParticle).final_angle_diff_score_component;
        // #522                debugInfoMessage.bestParticleScore     = (*bestParticle).getParticleScore();
        // #522                debugInfoMessage.overAllParticleScore = current_layoutScore;
        // #522
        // #522                //vector<LayoutComponent*> *vec = bestParticle->getLayoutComponentsPtr();
        // #522                //LayoutComponent_RoadState* lc  = dynamic_cast<LayoutComponent_RoadState*>(vec->at(0)); //JUST BECAUSE NOW WE HAVE ONLY ONE COMPONENT
        // #522                LayoutComponent_RoadState* lc  = bestParticle->giveMeThatComponent<LayoutComponent_RoadState>(); // refs #503
        // #522                if (lc)
        // #522                {
        // #522                    debugInfoMessage.scoreRoadLane_Lanes   = lc->getScoreLanes();
        // #522                    debugInfoMessage.scoreRoadLane_Width   = lc->getScoreWidth();
        // #522                    debugInfoMessage.scoreRoadLane_Total   = lc->getTotalComponentScore();
        // #522                }
        // #522
        // #522                this->publisher_debugInformation.publish(debugInfoMessage);
        // #522            }
        // #522        }
        // #522        catch (...)
        // #522        {
        // #522            ROS_ERROR_STREAM("Something went wrong in the publising debug information " << __LINE__ << " of " << __FILE__);
        // #522        }



        // ------------------------------------------------------------------------------- //

        // ------------------------------ resampling ------------------------------------- //
        if (new_detections)
        {
            //          Add new candidate-components ----------------------------------------------------- //
            //          (1) given N new candidate-layout-components duplicate 2^N particles and those particles to them
            //          (2) calculate score
            //          (3) resample all combination
        }


        pcl::console::TicToc ttresampling;
        // RESAMPLING --------------------------------------------------------------------------------------------------------------------------
        if (resampling_count++ == resampling_interval)
            //    if(0)
        {
            ttresampling.tic();
            ROS_DEBUG_STREAM("> Begin resampling phase!");
            resampling_count = 0;
            //ROS_DEPRECATED vector<Particle> new_current_layout;
            vector<shared_ptr <Particle>> new_current_layout_shared;
            new_current_layout_shared.reserve(current_layout_shared.size());
            vector<double> particle_score_vect;
            double cum_score_sum = 0.0f;

            // Build cumulative sum of the score and roulette vector
            for (particle_itr = current_layout_shared.begin(); particle_itr != current_layout_shared.end(); particle_itr++ )
            {
                cum_score_sum += (*particle_itr)->getParticleScore();
                particle_score_vect.push_back(cum_score_sum);
            }

            ROS_DEBUG_STREAM(" Resampling: sum of all particles scores: " << cum_score_sum);

            if (cum_score_sum != 0)
            {
                // Init uniform distribution
                boost::uniform_real<double> uniform_rand(0, cum_score_sum);
                boost::uniform_real<double> uniform_rand2(0, 100);
                boost::uniform_real<double> uniform_rand3(0, current_layout_shared.size());

                // find weight that is at least num
                vector<double>::iterator score_itr;
                //shared_ptr<Particle> temp_part;

                clock_t tic = clock();
                int ddd = 0;
                for (int k = 0; k < current_layout_shared.size(); ++k)
                {
                    if (uniform_rand2(rng) <= 95) //This percentage of weighted samples
                    {
                        // WEIGHTED RESAMPLER
                        int particle_counter = 0;
                        clock_t ran1 = clock();
                        double num = uniform_rand(rng);
                        clock_t ran2 = clock();

                        clock_t tic = clock();
                        ddd = 0;
                        for (score_itr = particle_score_vect.begin(); score_itr != particle_score_vect.end(); score_itr++ )
                        {
                            if ( *score_itr >= num)
                            {
                                shared_ptr<Particle> temp_part(new Particle(*current_layout_shared.at(particle_counter))); //#586
                                //temp_part.reset(new Particle(node_handle,tf_listener, *current_layout_shared.at(particle_counter))); //#586
                                temp_part->setId(k);
                                temp_part->setParticleScore(1.0f);
                                //                            stat_out_file << temp_part.getId() << "\t";
                                //                            particle_poses_statistics(k,0) = (temp_part.getParticleState().getPosition())(0);
                                //                            particle_poses_statistics(k,1) = (temp_part.getParticleState().getPosition())(1);
                                new_current_layout_shared.push_back(temp_part);
                                break;
                            }
                            ddd++;
                            particle_counter++;
                        }
                        clock_t toc = clock();
                        ROS_DEBUG("CREA LA PARTICELLA Elapsed: %f seconds\t#@%d\t\t RAN=%f \n", (double)(toc - tic) / CLOCKS_PER_SEC, ddd, (double)(ran1 - ran2) / CLOCKS_PER_SEC);
                    }
                    else
                    {
                        // UNIFORM RESAMPLER
                        // TODO: check if this sampler works (check the randomizer).
                        ROS_WARN_STREAM("Uniform sampler needs re-check!");
                        int temp_rand = floor(uniform_rand3(rng));
                        shared_ptr<Particle> temp_part(new Particle(*current_layout_shared.at(temp_rand)));
                        temp_part->setId(k);
                        temp_part->setParticleScore(1.0f);
                        new_current_layout_shared.push_back(temp_part);
                    }

                }
                clock_t toc = clock();
                ROS_DEBUG("ROULETTE Elapsed: %f seconds     USING total score equals to: %f", (double)(toc - tic) / CLOCKS_PER_SEC, cum_score_sum);

                // copy resampled particle-set
                current_layout_shared.clear();
                current_layout_shared = new_current_layout_shared;
                new_current_layout_shared.clear();
            }
            ROS_DEBUG_STREAM("< End resampling phase! \t" << ttresampling.toc());
        }
        // END RESAMPLING ----------------------------------------------------------------------------------------------------------------------

        /// Other
        LayoutManager::publishMarkerArray((*bestParticle).getParticleScore()); // L^infinity-Normalization, passing the normalization-factor (the best particle score)

    }
    else
    {
        cout << endl <<  "Not moved!" << endl;
        //TODO: calculate score again, with new detections (if available)

    }

    int64_t elapsedtime = tt2.toc();
    std_msgs::Int64 t;
    t.data = elapsedtime;
    RLETIME.publish(t);

    ROS_INFO_STREAM ("Exiting RLE layoutEstimation loop, the elapsed time was: \t" << elapsedtime);

    //return current_layout_shared; // current layout is the <vector> of Particles
}

ROS_DEPRECATED void LayoutManager::normalizeParticleSet()
{

    //Applies a L-infinity normalization

    vector<shared_ptr<Particle>>::iterator itr;

    double sum = 0.0f;
    double max = -1.0f;

    for (itr = current_layout_shared.begin(); itr != current_layout_shared.end(); itr++)
    {
        sum += (*itr)->getParticleScore();
        if ((*itr)->getParticleScore() > max)
            max = (*itr)->getParticleScore();
    }
    for (itr = current_layout_shared.begin(); itr != current_layout_shared.end(); itr++)
        (*itr)->setParticleScore((*itr)->getParticleScore() / max);
}




vector<shared_ptr<Particle> > LayoutManager::getCurrent_layout_shared() const
{
    return current_layout_shared;
}

void LayoutManager::setCurrent_layout_shared(const vector<shared_ptr<Particle> > &value)
{
    current_layout_shared = value;
}

//vector<Particle> LayoutManager::getCurrentLayout()
//{
//    return current_layout;
//}
//
//void LayoutManager::setCurrentLayout(vector<Particle> &p_set)
//{
//    current_layout = p_set;
//}



inline void latlon2xy_helper(double &lat, double &lngd)
{

    // WGS 84 datum
    double eqRad = 6378137.0;
    double flat = 298.2572236;

    // constants used in calculations:
    double a = eqRad;           // equatorial radius in meters
    double f = 1.0 / flat;        // polar flattening
    double b = a * (1.0 - f);     // polar radius
    double e = sqrt(1.0 - (pow(b, 2) / pow(a, 2))); // eccentricity
    double k0 = 0.9996;
    double drad = M_PI / 180.0;

    double phi = lat * drad;   // convert latitude to radians
    double utmz = 1.0 + floor((lngd + 180.0) / 6.0); // longitude to utm zone
    double zcm = 3.0 + 6.0 * (utmz - 1.0) - 180.0;     // central meridian of a zone
    double esq = (1.0 - (b / a) * (b / a));
    double e0sq = e * e / (1.0 - e * e);
    double M = 0.0;
    double M0 = 0.0;
    double N = a / sqrt(1.0 - pow(e * sin(phi), 2));
    double T = pow(tan(phi), 2);
    double C = e0sq * pow(cos(phi), 2);
    double A = (lngd - zcm) * drad * cos(phi);

    // calculate M (USGS style)
    M = phi * (1.0 - esq * (1.0 / 4.0 + esq * (3.0 / 64.0 + 5.0 * esq / 256.0)));
    M = M - sin(2.0 * phi) * (esq * (3.0 / 8.0 + esq * (3.0 / 32.0 + 45.0 * esq / 1024.0)));
    M = M + sin(4.0 * phi) * (esq * esq * (15.0 / 256.0 + esq * 45.0 / 1024.0));
    M = M - sin(6.0 * phi) * (esq * esq * esq * (35.0 / 3072.0));
    M = M * a; // Arc length along standard meridian

    // now we are ready to calculate the UTM values...
    // first the easting (relative to CM)
    double x = k0 * N * A * (1.0 + A * A * ((1.0 - T + C) / 6.0 + A * A * (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * e0sq) / 120.0));
    x = x + 500000.0; // standard easting

    // now the northing (from the equator)
    double y = k0 * (M - M0 + N * tan(phi) * (A * A * (1.0 / 2.0 + A * A * ((5.0 - T + 9.0 * C + 4.0 * C * C) / 24.0 + A * A * (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * e0sq) / 720.0))));
    if (y < 0)
    {
        y = 10000000.0 + y; // add in false northing if south of the equator
    }
    double easting = round(10.0 * x) / 10.0;
    double northing = round(10.0 * y) / 10.0;


    lat = easting;
    lngd = northing;


    // the following lines were added to debug the rviz_satellite with Bing Maps. You can delete when needed.
    //double sinLatitude = sin(latitude * M_PI/180);
    //coords.x = ((longitude + 180.0f) / 360.0f) * 256.0f * pow(2,zoom);
    //coords.y = (0.5f-log((1.0f+sinLatitude) / (1.0f-sinLatitude)) / (4.0f*M_PI)) * 256.0f * pow(2,zoom);

    //double sinLatitude = sin(lat * M_PI/180);
    //ROS_ERROR_STREAM ("DIFFERENCE\t"<<coords.x << " " <<((lngd + 180.0f) / 360.0f) * 256.0f * pow(2,19));
    //ROS_ERROR_STREAM ("DIFFERENCE\t"<<coords.y << " " <<(0.5f-log((1.0f+sinLatitude) / (1.0f-sinLatitude)) / (4.0f*M_PI)) * 256.0f * pow(2,19));

}

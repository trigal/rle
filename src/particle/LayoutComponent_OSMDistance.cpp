#include "LayoutComponent_OSMDistance.h"
#include "Particle.h"


double LayoutComponent_OSMDistance::getAlphas()
{
    return street_distribution_alpha + angle_distribution_alpha;
}

void LayoutComponent_OSMDistance::calculateComponentScore()
{
    // get PDF score FOR DISTANCE
    boost::math::normal normal_dist(0, this->street_distribution_sigma);
    this->pose_diff_score_component = pdf(normal_dist, this->distance_to_closest_segment) / pdf(normal_dist , 0.0f); //normalized to 1

    // get PDF score from first angle
    double first_angle_difference  = Utils::normalize_angle(first_quaternion_diff.getAngle());
    double second_angle_difference = Utils::normalize_angle(second_quaternion_diff.getAngle());

    boost::math::normal angle_normal_dist(0, this->angle_distribution_sigma);
    double first_angle_diff_score = pdf(angle_normal_dist, first_angle_difference) / pdf(angle_normal_dist, 0.0f);

    this->final_angle_diff_score_component = 0.0f; //set in both parts of the IF, resetted here

    //      if street have 2 directions check angle diff with opposite angle
    //if (this->snapParticle_serviceMessage.response.way_dir_opposite_particles) //cazzomene controlla cmq
    if (1)
    {

        //      get PDF score
        double second_angle_diff_score = pdf(angle_normal_dist, second_angle_difference) / pdf(angle_normal_dist, 0.0f); //TODO: refs #442 check if this normalization is correct or I used the opposite angle

        //      set score
        if (second_angle_diff_score > first_angle_diff_score)
        {
            this->final_angle_diff_score_component = second_angle_diff_score;
            this->misalignment_to_closest_segment = second_angle_difference;
        }
        else
        {
            this->final_angle_diff_score_component = first_angle_diff_score;
            this->misalignment_to_closest_segment = first_angle_difference;
        }

        ROS_DEBUG_STREAM("ONEWAY-No \tSPATIAL_DISTANCE: " << this->distance_to_closest_segment << "\tANGLE_1: " << first_angle_difference << "\tANGLE_2: " << second_angle_difference);
    }
    else
    {
        ROS_DEBUG_STREAM("ONEWAY-Yes\tSPATIAL_DISTANCE: " << this->distance_to_closest_segment << "\tANGLE_1: " << first_angle_difference);
        this->final_angle_diff_score_component = first_angle_diff_score;
    }


    /// This is the final weight of the component
    this->component_weight = this->street_distribution_alpha * this->pose_diff_score_component          +
                             this->angle_distribution_alpha  * this->final_angle_diff_score_component;


}

void LayoutComponent_OSMDistance::componentPerturbation()
{

}

void LayoutComponent_OSMDistance::componentPoseEstimation(int index)
{
    pcl::console::TicToc tt,tt2,tt3,tt4;
    tt.tic();

    //ROS_INFO_STREAM("Entering > " << __PRETTY_FUNCTION__);
    /*
     *  First phase, coming from Sampling >
     *                              PropagateLayoutComponents >
     *                                  componentPoseEstimation (this one)
     *
     *  Here we do what was done before inside CalculateGeometricScores
     *
     */


    // Get particle state
    tt2.tic();
    Vector3d particle_position_state = this->particlePtr->getParticleState().getPosition();
    geometry_msgs::PoseStamped pose_local_map_frame;
    pose_local_map_frame.header.frame_id = "local_map";
    pose_local_map_frame.header.stamp = ros::Time::now();
    pose_local_map_frame.pose.position.x = particle_position_state(0);
    pose_local_map_frame.pose.position.y =  particle_position_state(1);
    pose_local_map_frame.pose.position.z =  particle_position_state(2);
    Eigen::Quaterniond particle_orientation_state_quaterion(this->particlePtr->getParticleState().getRotation());
    pose_local_map_frame.pose.orientation.w = particle_orientation_state_quaterion.w();
    pose_local_map_frame.pose.orientation.x = particle_orientation_state_quaterion.x();
    pose_local_map_frame.pose.orientation.y = particle_orientation_state_quaterion.y();
    pose_local_map_frame.pose.orientation.z = particle_orientation_state_quaterion.z();

    tf::Stamped<tf::Pose> tf_pose_map_frame, tf_pose_local_map_frame;
    tf::poseStampedMsgToTF(pose_local_map_frame, tf_pose_local_map_frame);
    // Transform pose from "local_map" to "map"
    try
    {
        tf_listener_->waitForTransform("map", "local_map", ros::Time(0), ros::Duration(0.5)); // TODO: Feature #588
        tf_listener_->transformPose("map", ros::Time(0), tf_pose_local_map_frame, "local_map", tf_pose_map_frame);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << "\t" << "%s" << ex.what());
        ROS_ERROR_STREAM("     Transform snapped particle pose from local_map to map");
        ros::shutdown();    // TODO: handle this, now shutdown requested. augusto debug
        return;
    }
    ROS_DEBUG_STREAM("tt2\t" << tt2.toc());


    // Build request for getting snapped XY values + orientation of the road    
    ira_open_street_map::snap_particle_xy snapParticle_serviceMessage;
    snapParticle_serviceMessage.request.x = tf_pose_map_frame.getOrigin().getX();
    snapParticle_serviceMessage.request.y = tf_pose_map_frame.getOrigin().getY();
    snapParticle_serviceMessage.request.max_distance_radius = 100; // distance radius for finding the closest nodes for particle snap WARNING: parametrize this value
        // 100 per lo snap una volta funzionante?!!?!?!?!? DIOSANTO
    ros::ServiceClient snap_particle_xy_client;
    snap_particle_xy_client                 = node_handler.serviceClient<ira_open_street_map::snap_particle_xy>("/ira_open_street_map/snap_particle_xy");

    printf("> going to ask\n");
    tt4.tic();
    // Get distance from snapped particle pose and set it as particle score
    if (snap_particle_xy_client.call(snapParticle_serviceMessage))
    {
        printf("< got an answer\n");
        ROS_DEBUG_STREAM("snap particle : tt4\t" << tt4.toc());
        /*
         * This refs #538 . Checks if the particle is on the left(1) or right(0)
         *
         *       TRUE   FALSE
         *     |      .      |
         *     |      .  d1  |
         *     |      .---*  |
         *     |      .      |
         *     |  *---.      |
         *          d2
         *
         * this is calculated inside osm_query_node.cpp - snap_particle_xy
         */

        tt3.tic();
        this->isLeft = snapParticle_serviceMessage.response.isLeft; // this refs #538

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


        // Transform pose from "map" to "local_map"
        try
        {
            tf_listener_->waitForTransform("local_map", "map", ros::Time(0), ros::Duration(0.5)); // TODO: Feature #588
            tf_listener_->transformPose("local_map", ros::Time(0), tf_snapped_map_frame, "map", tf_snapped_local_map_frame);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR_STREAM("RLE MAIN LOOP");
            ROS_ERROR_STREAM("%s" << ex.what());
            ROS_ERROR_STREAM("     Transform snapped particle pose from map to local_map");
            ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
            return;
        }



        // calculate distance from original particle positin and snapped particle position ---------------------------------
        // use it for score calculation with normal distribution PDF
        double dx       = tf_pose_local_map_frame.getOrigin().getX() - tf_snapped_local_map_frame.getOrigin().getX();
        double dy       = tf_pose_local_map_frame.getOrigin().getY() - tf_snapped_local_map_frame.getOrigin().getY();
        double dz       = tf_pose_local_map_frame.getOrigin().getZ() - 0; // particle Z axis is forced to be next to zero
        double distance = sqrt(dx * dx + dy * dy + dz * dz);

        /// update the particle variable; this is also used in the
        /// CREATING STATISTICS FOR RLE OUTPUT section
        //(*particle_itr).distance_to_closest_segment = distance;
        this->distance_to_closest_segment = distance;


        ROS_DEBUG_STREAM("tt3\t" << tt3.toc());


        // calculate QUATERNIONE difference for both cases
        this->first_quaternion_diff = tf_snapped_local_map_frame.getRotation().inverse() * tf_pose_local_map_frame.getRotation();

        tf_snapped_local_map_frame_opposite_direction = tf_snapped_local_map_frame; // COPY TRANSFORM (I PRAY FOR THIS)
        tf_snapped_local_map_frame_opposite_direction.setRotation(tf_snapped_local_map_frame * tf::createQuaternionFromYaw(M_PI)); // INVERT DIRECTION
        this->second_quaternion_diff = tf_snapped_local_map_frame_opposite_direction.getRotation().inverse() * tf_pose_local_map_frame.getRotation();




    }
    else
    {
        // Either service is down or particle is too far from a street
        //(*particle_itr).setParticleScore(0);
        this->distance_to_closest_segment       = std::numeric_limits<double>::infinity();
        this->misalignment_to_closest_segment   = std::numeric_limits<double>::infinity();

        ROS_ERROR_STREAM(__PRETTY_FUNCTION__);
        ROS_ERROR_STREAM("RLE Main loop, snap_particle_xy service call");
        ROS_ERROR_STREAM("Either service is down or particle is too far from a street. Shutdown in LayoutManager.cpp");
        //ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
    }// end snap particle client



    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << " %%%555 index= "<< index <<"\t" << tt.toc());

}

double LayoutComponent_OSMDistance::getDistance_to_closest_segment() const
{
    return distance_to_closest_segment;
}

void LayoutComponent_OSMDistance::setDistance_to_closest_segment(double value)
{
    distance_to_closest_segment = value;
}

bool LayoutComponent_OSMDistance::getIsLeft() const
{
    return isLeft;
}

void LayoutComponent_OSMDistance::setIsLeft(bool value)
{
    isLeft = value;
}

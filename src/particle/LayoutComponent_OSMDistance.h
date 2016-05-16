#ifndef LAYOUTCOMPONENT_OSMDISTANCE_H
#define LAYOUTCOMPONENT_OSMDISTANCE_H

#include <algorithm>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <limits>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vector>

#include "LayoutComponent.h"
#include "../Utils.h"
#include "ira_open_street_map/snap_particle_xy.h"

using namespace std;


/**
 * @brief The LayoutComponent_OSMDistance
 * @author Augusto Luis Ballardini
 * @date December 2015
 *
 * This is the answer to Task #522, use the GeometricScores as a real component.
 * Creating a component for the two distances
 */

class LayoutComponent_OSMDistance : public LayoutComponent
{

private:

    double distance_to_closest_segment;     ///< Euclidean distance, meters
    double misalignment_to_closest_segment; ///< Angular misalignment, radians
    bool   isLeft;                            ///< stores if the the particle is on the left or right to the nearest road segment refs #538

    double street_distribution_sigma;       ///< Sigma used for the normal distribution
    double angle_distribution_sigma ;       ///< Sigma used for the normal distribution
    double street_distribution_alpha;      ///< Fraction of the weigth #534
    double angle_distribution_alpha ;      ///< Fraction of the weigth #534

    /* The two following parameters were calculated inside the
     * LayoutManager::calculateGeometricScores and then evaluated inside the
     * LayoutManager::LayoutEstimation
     */
    double pose_diff_score_component;        // have a look to #522
    double final_angle_diff_score_component; // have a look to #522

    // Diff quaternions used for angle score
    tf::Quaternion first_quaternion_diff;
    tf::Quaternion second_quaternion_diff;

    // ROS node handler and TF listener needed to look up for transforms.
    ros::NodeHandle node_handler;            // WARNING: this is a copy?! seems to be ok ... http://answers.ros.org/question/12375/is-it-possible-to-have-2-nodehandlers-for-a-single-node/
    tf::TransformListener tf_listener;

    /// The ROS-Service declaration. Initialization in the constructor.
    ros::ServiceClient snap_particle_xy_client;

    /// The message/service used to call/receive info from the service above
    ira_open_street_map::snap_particle_xy snapParticle_serviceMessage;

public:

    /**
     * @brief getAlphas
     * @return street_distribution_alpha + angle_distribution_alpha
     */
    double getAlphas();


    /**
     * @brief calculateComponentScore
     * Implementation of pure virtual method 'calculateWeight'
     */
    void calculateComponentScore();

    /**
     * @brief componentPerturbation
     * Implementation of pure virtual method 'componentPerturbation'
     */
    void componentPerturbation();

    /**
     * @brief componentPoseEstimation
     * Implementation of pure virtual method 'componentPoseEstimation'
     *
     * called from this tree:
     *      2. Particle::propagateLayoutComponents()
     *      1. LayoutManager::sampling()
     */
    void componentPoseEstimation();

    /**
     * @brief LayoutComponent_RoadLane
     * Default Constructor
     */

    LayoutComponent* clone()
    {
        LayoutComponent* cloned = new LayoutComponent_OSMDistance(particle_id, component_id, distance_to_closest_segment,
                                                                  final_angle_diff_score_component, street_distribution_sigma,
                                                                  angle_distribution_sigma, street_distribution_alpha, angle_distribution_alpha);

        dynamic_cast<LayoutComponent_OSMDistance*>(cloned)->pose_diff_score_component = this->pose_diff_score_component;
        dynamic_cast<LayoutComponent_OSMDistance*>(cloned)->first_quaternion_diff = this->first_quaternion_diff;
        dynamic_cast<LayoutComponent_OSMDistance*>(cloned)->second_quaternion_diff = this->second_quaternion_diff;
        dynamic_cast<LayoutComponent_OSMDistance*>(cloned)->snapParticle_serviceMessage = this->snapParticle_serviceMessage;
        return cloned;
    }

    LayoutComponent_OSMDistance(const unsigned int particle_id,
                                const unsigned int component_id,
                                const double euclideanDistanceMeters,
                                const double angularDistanceRAD,
                                const double street_distribution_sigma,
                                const double angle_distribution_sigma,
                                const double street_distribution_weight,
                                const double angle_distribution_weight
                               )
    {
        ROS_INFO_STREAM(__PRETTY_FUNCTION__);
        this->particle_id                       = particle_id;
        this->component_id                      = component_id; // this refs #525
        this->distance_to_closest_segment       = euclideanDistanceMeters;
        this->final_angle_diff_score_component  = angularDistanceRAD;
        this->street_distribution_sigma         = street_distribution_sigma;
        this->angle_distribution_sigma          = angle_distribution_sigma ;
        this->street_distribution_alpha         = street_distribution_weight;
        this->angle_distribution_alpha          = angle_distribution_weight ;

        snap_particle_xy_client                 = node_handler.serviceClient<ira_open_street_map::snap_particle_xy>("/ira_open_street_map/snap_particle_xy");

        pose_diff_score_component               = 0.0f;
        final_angle_diff_score_component        = 0.0f;

        ROS_ASSERT(this->street_distribution_alpha   > 0.0f);
        ROS_ASSERT(this->angle_distribution_alpha    > 0.0f);
    }

    double getDistance_to_closest_segment() const;
    void setDistance_to_closest_segment(double value);
    bool getIsLeft() const;
    void setIsLeft(bool value);
};

#endif // LAYOUTCOMPONENT_OSMDISTANCE_H

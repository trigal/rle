#ifndef LAYOUTCOMPONENT_ROADLANE_H
#define LAYOUTCOMPONENT_ROADLANE_H

#include <algorithm>    // std::sort
#include <eigen3/Eigen/Core>
#include <iostream>
#include <limits>
#include <ros/ros.h>
#include <vector>       // std::vector

#include "LayoutComponent.h"
#include "../Utils.h"
#include "road_layout_estimation/msg_lines.h"
#include "road_layout_estimation/msg_lineInfo.h"

using namespace std;

/**
 * @brief The LayoutComponent_RoadLane class
 *
 * This component should answer to the following question: IN WHICH LANE I AM?
 *
 * second component, Alcala de Henares 2015
 *
 */
class LayoutComponent_RoadLane : public LayoutComponent
{

private:

    Eigen::VectorXd sensor;
    Eigen::ArrayXd megavariabile;

    bool myCompare(road_layout_estimation::msg_lineInfo a, road_layout_estimation::msg_lineInfo b); ///< Function used to sort the list of lines in the array, using std::sort

public:

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
     * Implementation of pure virtual method 'componentPerturbation'
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
    LayoutComponent_RoadLane()
    {
        ROS_INFO_STREAM(__PRETTY_FUNCTION__);
    }

    LayoutComponent_RoadLane(const unsigned int particle_id, const unsigned int component_id,unsigned int lanes)
    {
        ROS_INFO_STREAM(__PRETTY_FUNCTION__);

        // Resize the State and the SensorModel
        megavariabile.resize(lanes);
        sensor.resize(lanes);

        megavariabile.setConstant(1.0f/double(lanes));

        this->particle_id  = particle_id;
        this->component_id = component_id;
    }

};

#endif // LAYOUTCOMPONENT_ROADLANE_H

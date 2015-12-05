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
 * http://projects.ira.disco.unimib.it/issues/527
 *
 */
class LayoutComponent_RoadLane : public LayoutComponent
{

private:

    Eigen::VectorXd sensor;
    Eigen::ArrayXd  megavariabile;
    Eigen::MatrixXd stateTransitionMatrix;

    int howManyLanes;                   ///< The number of lanes in the current hypothesis
    double standardLaneWidth = 3.0f;    // maybe minLaneWidth
    int MAX_COUNT = 10;                 ///< Counter upper limit -- this value need to be the same of RoadMarks.h

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
        setLanes(0);
    }

    LayoutComponent_RoadLane(const unsigned int particle_id, const unsigned int component_id, unsigned int lanes)
    {
        ROS_INFO_STREAM(__PRETTY_FUNCTION__);

        // Set component variables
        //resetMegavariabile(lanes);
        //resetSensor(lanes);
        //setLanes(lanes);
        //setStateTransitionMatrix();

        setLanes(2);
        resetMegavariabile();
        resetSensor();
        setStateTransitionMatrix();

        // Set inherited variables (LayoutComponent)
        this->particle_id  = particle_id;
        this->component_id = component_id;
    }

    /**
     * @brief filter
     * @param msg_lines
     *
     * This is the porting of the test-function: fake_callback.cpp
     */
    void filter(const road_layout_estimation::msg_lines & msg_lines);

    /**
     * @brief resetSensor
     * @param lanes, if zero, getLanes will be used
     * This function resets the Sensor matrix
     */
    void resetSensor(int howManyLanes = 0);

    /**
     * @brief resetMegavariabile
     * @param lanes, if zero, getLanes will be used
     * This function resets the Megavariabile
     */
    void resetMegavariabile(int howManyLanes = 0);

    void setStateTransitionMatrix();

    int getLanes() const;
    void setLanes(int value);
};

#endif // LAYOUTCOMPONENT_ROADLANE_H

#ifndef LAYOUTCOMPONENT_ROADLANE_H
#define LAYOUTCOMPONENT_ROADLANE_H

#include "LayoutComponent.h"
#include <iostream>
#include <ros/ros.h>

#include "../Utils.h"

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

    std::vector <double> laneP;

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
        laneP.clear();
    }
};

#endif // LAYOUTCOMPONENT_ROADLANE_H

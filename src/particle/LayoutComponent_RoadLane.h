#ifndef LAYOUTCOMPONENT_ROADLANE_H
#define LAYOUTCOMPONENT_ROADLANE_H

#include "LayoutComponent.h"
#include <iostream>
#include <ros/ros.h>

using namespace std;

///
/// \brief The LayoutComponent_RoadLane class
///
class LayoutComponent_RoadLane : public LayoutComponent
{

private:

public:

    ///
    /// \brief calculateComponentScore
    /// Implementation of pure virtual method 'calculateWeight'
    void calculateComponentScore()
    {
        cout << "Calculating weight of ROAD LANE component ID: " << component_id << " that belongs to particle ID: " << particle_id << endl;
    }

    ///
    /// \brief componentPerturbation
    /// Implementation of pure virtual method 'componentPerturbation'
    void componentPerturbation()
    {
        cout << "Perturbating ROAD LANE component ID: " << component_id << " that belongs to particle ID: " << particle_id << endl;
    }

    ///
    /// \brief componentPoseEstimation
    /// Implementation of pure virtual method 'componentPerturbation'
    ///
    void componentPoseEstimation()
    {
        cout << "Propagating and estimating ROAD LANE component pose. ID: " << component_id << " that belongs to particle ID: " << particle_id << endl;
    }


};

#endif // LAYOUTCOMPONENT_ROADLANE_H

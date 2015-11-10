#include "LayoutComponent_RoadLane.h"

void LayoutComponent_RoadLane::calculateComponentScore()
{
    cout << "Calculating weight of ROAD LANE component ID: " << component_id << " that belongs to particle ID: " << particle_id << endl;
}

void LayoutComponent_RoadLane::componentPerturbation()
{
    cout << "Perturbating ROAD LANE component ID: " << component_id << " that belongs to particle ID: " << particle_id << endl;
}

void LayoutComponent_RoadLane::componentPoseEstimation()
{
    cout << "Propagating and estimating ROAD LANE component pose. ID: " << component_id << " that belongs to particle ID: " << particle_id << endl;
}

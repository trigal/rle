#include "LayoutComponent_RoadLane.h"

bool LayoutComponent_RoadLane::myCompare(road_layout_estimation::msg_lineInfo a, road_layout_estimation::msg_lineInfo b)
{
    if (a.offset > b.offset)
        //if (std::fabs(a.offset) < std::fabs(b.offset))
        return true;
    else
        return false;
}

void LayoutComponent_RoadLane::calculateComponentScore()
{
    ROS_DEBUG_STREAM ("Calculating weight of ROAD LANE component ID: " << component_id << " that belongs to particle ID: " << particle_id);
}

void LayoutComponent_RoadLane::componentPerturbation()
{
    ROS_DEBUG_STREAM ("Perturbating ROAD LANE component ID: " << component_id << " that belongs to particle ID: " << particle_id);
}

/**
 * @brief LayoutComponent_RoadLane::componentPoseEstimation
 *
 * Disabled right now. Here we should update the detected lines using the
 * ego-motion and weighting again everything. With the ISIS lane detector
 * this is done inside the detector and I don't know how to change things
 * there.
 */
void LayoutComponent_RoadLane::componentPoseEstimation()
{
    ROS_DEBUG_STREAM ("Propagating and estimating ROAD LANE component pose. ID: " << component_id << " that belongs to particle ID: " << particle_id );
}

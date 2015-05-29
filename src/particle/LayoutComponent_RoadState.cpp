#include "LayoutComponent_RoadState.h"

/**
 * Implementation of pure virtual method 'calculateWeight'
 */
void LayoutComponent_RoadState::calculateComponentScore(){
    cout << "Calculating weight of ROAD LANE component ID: " << component_id << " that belongs to particle ID: " <<particle_id << endl;
}

/**
 * Implementation of pure virtual method 'componentPerturbation'
 */
void LayoutComponent_RoadState::componentPerturbation(){
    cout << "Perturbating ROAD LANE component ID: " << component_id << " that belongs to particle ID: " <<particle_id << endl;
}

/**
 * Implementation of pure virtual method 'componentPerturbation'
 */
void LayoutComponent_RoadState::componentPoseEstimation(){
    cout << "Propagating and estimating ROAD LANE component pose. ID: " << component_id << " that belongs to particle ID: " <<particle_id << endl;
}



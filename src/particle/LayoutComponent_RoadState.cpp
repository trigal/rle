#include "LayoutComponent_RoadState.h"

/**
 * Implementation of pure virtual method 'calculateWeight'
 */

double LayoutComponent_RoadState::getRoad_width() const
{
    return road_width;
}

void LayoutComponent_RoadState::setRoad_width(double value)
{
    road_width = value;
}

char LayoutComponent_RoadState::getCurrent_lane() const
{
    return current_lane;
}

void LayoutComponent_RoadState::setCurrent_lane(char value)
{
    current_lane = value;
}

unsigned char LayoutComponent_RoadState::getLanes_number() const
{
    return lanes_number;
}

void LayoutComponent_RoadState::setLanes_number(unsigned char value)
{
    lanes_number = value;
}

int LayoutComponent_RoadState::getWay_id() const
{
    return way_id;
}

void LayoutComponent_RoadState::setWay_id(const int &value)
{
    way_id = value;
}
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



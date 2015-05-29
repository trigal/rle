#ifndef LAYOUTCOMPONENT_ROADSTATE_H
#define LAYOUTCOMPONENT_ROADSTATE_H

#include "LayoutComponent.h"
#include <iostream>
#include <ros/ros.h>

using namespace std;

class LayoutComponent_RoadState : public LayoutComponent
{
private:

    unsigned char   lanes_number;
    unsigned char   current_lane;
    double          road_width;
    bool            oneway;

    ros::Time timestamp;

public:

    void calculateComponentScore();
    void componentPerturbation();
    void componentPoseEstimation();

    // Getters and setters ----------------------------------------------------------------------
    void setLanesNumber(unsigned char val){ lanes_number = val; }
    void setCurrentLane(unsigned char val){ current_lane = val; }
    void setRoadWidth(double val){ road_width = val; }
    void setTimestamp(ros::Time time) { timestamp = time; }
    ros::Time getTimestamp() { return timestamp; }

    // Constructors and destructors -------------------------------------------------------------
    LayoutComponent_RoadState(){
        particle_id = 0;
        component_id = 0;
        component_weight = 0;
        component_state = VectorXd::Zero(12);
        component_cov = MatrixXd::Zero(12,12);
        lanes_number = 0;
        current_lane = 0;
        road_width = 0.0f;
        timestamp = ros::Time(0);
    }
    LayoutComponent_RoadState(const unsigned int p_id, const unsigned int c_id, const unsigned char lanes_number, const unsigned char current_lane, double road_width, ros::Time timestamp){
        particle_id = p_id;
        component_id = c_id;
        component_weight = 0;
        component_state = VectorXd::Zero(12);
        component_cov = MatrixXd::Zero(12,12);
        this->lanes_number= lanes_number;
        this->current_lane = current_lane;
        this->road_width = road_width;
        this->timestamp = timestamp;
    }
    ~LayoutComponent_RoadState(){
        particle_id = 0;
        component_id = 0;
        component_weight = 0;
        component_state.resize(0);
        component_cov.resize(0,0);
        timestamp = ros::Time(0);
    }
};

#endif // LAYOUTCOMPONENT_ROADSTATE_H

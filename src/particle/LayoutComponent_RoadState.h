#ifndef LAYOUTCOMPONENT_ROADSTATE_H
#define LAYOUTCOMPONENT_ROADSTATE_H

#include "LayoutComponent.h"

#include <boost/math/distributions/normal.hpp>
#include <boost/math/distributions/poisson.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include "../Utils.h"

#include "ira_open_street_map/getHighwayInfo.h"

#include <iostream>
#include <ros/ros.h>

using namespace std;

class LayoutComponent_RoadState : public LayoutComponent
{
private:

    int   lanes_number;
    char            current_lane;     // -1 don't know
    double          road_width;
    bool            oneway;
    int             way_id;

    ros::ServiceClient *getHighwayInfo_client;

    // Particle info
    double          latitude;
    double          longitude;

    ros::Time timestamp;

public:

    void componentPoseEstimation();
    void calculateComponentScore();
    void componentPerturbation();

    // Getters and setters ----------------------------------------------------------------------
    void          setTimestamp(ros::Time time)  { timestamp = time; }
    ros::Time     getTimestamp()                { return timestamp; }
    double        getRoad_width()               const;
    char          getCurrent_lane()             const;
    int           getLanes_number()             const;
    void          setLanes_number               (int  value);
    void          setRoad_width                 (double value);
    void          setCurrent_lane               (char value);

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

        getHighwayInfo_client=NULL;
    }
    LayoutComponent_RoadState(const unsigned int particle_id, const unsigned int component_id, int way_id, int lanes_number, const unsigned char current_lane, double road_width, ros::Time timestamp, ros::ServiceClient *serviceClientFromLayoutManager)
    {
        this->particle_id = particle_id;
        this->component_id = component_id;
        this->lanes_number= lanes_number;
        this->current_lane = current_lane;
        this->road_width = road_width;
        this->timestamp = timestamp;
        this->way_id=way_id;

        this->component_weight = 0;
        this->component_state = VectorXd::Zero(12);
        this->component_cov = MatrixXd::Zero(12,12);

        getHighwayInfo_client=serviceClientFromLayoutManager;
    }

    ~LayoutComponent_RoadState()
    {
        particle_id = 0;
        component_id = 0;
        component_weight = 0;
        component_state.resize(0);
        component_cov.resize(0,0);
        timestamp = ros::Time(0);
    }
    int getWay_id() const;
    void setWay_id(const int &value);
};

#endif // LAYOUTCOMPONENT_ROADSTATE_H

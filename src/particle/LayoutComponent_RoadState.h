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
#include "road_layout_estimation/msg_lines.h"

#include <iostream>
#include <ros/ros.h>

using namespace std;

class LayoutComponent_RoadState : public LayoutComponent
{
private:

    char            current_lane;     // -1 don't know

    //int             lanes_number;
    //double          road_width;
    //bool            oneway;
    //int64_t         way_id;

    road_layout_estimation::msg_lines msg_lines;

    ros::ServiceClient *getHighwayInfo_client;

    // Particle info
    double          latitude;
    double          longitude;

    ros::Time       timestamp;

    int linesFromLanes(int number_of_lanes);
    int lanesFromLines(int goodLines);

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
        //lanes_number = 0;
        //current_lane = 0;
        //road_width = 0.0f;
        //way_id = 0;
        timestamp = ros::Time(0);

        getHighwayInfo_client=NULL;
    }

    /// This constructor should be used only in the initialization phase
    ROS_DEPRECATED LayoutComponent_RoadState(const unsigned int particle_id, const unsigned int component_id, int way_id, int lanes_number, const unsigned char current_lane, double road_width, ros::Time timestamp, ros::ServiceClient *serviceClientFromLayoutManager)
    {
        this->particle_id = particle_id;
        this->component_id = component_id;
        this->timestamp = timestamp;
        this->current_lane = current_lane;

        //this->lanes_number= lanes_number;
        //this->road_width = road_width;
        //this->way_id=way_id;


        this->component_weight = 0;
        this->component_state = VectorXd::Zero(12);
        this->component_cov = MatrixXd::Zero(12,12);

        getHighwayInfo_client=serviceClientFromLayoutManager;
    }

    /// This constructor should be used during the normal filter iteration
    LayoutComponent_RoadState(const unsigned int particle_id, const unsigned int component_id, ros::Time timestamp, ros::ServiceClient *serviceClientFromLayoutManager, const road_layout_estimation::msg_lines &msg_lines)
    {
        this->particle_id   = particle_id;
        this->component_id  = component_id;
        this->timestamp     = timestamp;
        this->current_lane  = 0;                // current lane is not set.
        this->msg_lines     = msg_lines;

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
    int64_t getWay_id() const;
    void setWay_id(const int64_t &value);
};

#endif // LAYOUTCOMPONENT_ROADSTATE_H

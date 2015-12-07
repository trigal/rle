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
#include "road_layout_estimation/msg_lineInfo.h"

#include <iostream>
#include <ros/ros.h>

using namespace std;

/**
 * @brief The LayoutComponent_RoadState class
 *
 * This component should answer to the following question:
 *
 * QUANDO IL DETECTOR DI LINEE CI PERMETTE DI DIRE CHE LA 'DETECTION' SI AVVCINA
 * A QUANTO DICE OSM UTILIZZANDO
 *
 *               <<< N#LINEE + LARGHEZZA STRADA >>>
 *
 * first component, Alcala de Henares 2015
 * http://projects.ira.disco.unimib.it/issues/404
 *
 * The state is stored using a msg_lines message
 */
class LayoutComponent_RoadState : public LayoutComponent
{
private:

    char            current_lane_wrt_OSM_distance;  ///< here the lane in which the hypothesis should be wrt OSM lat/lon;
    bool            oneway;                         ///< oneway, retrieved using getHighwayService service

    // Description msg_lines
    // Header header
    // float64         number_of_lines  #adaptiveness of the filter! number of lanes that we were looking for
    // float64         goodLines        #how many good lines are currently found
    // float64         width            #full road width, calculated with valid line offsets
    // float64         naive_width      #full road width, calculated with all line offsets (even invalid)
    // ###int32           oneway           #MAKE SENSE? nope, moved to LayoutComponent_RoadState as class parameter
    // int64           way_id           #FOR COMPATIBILITY WITH roadStateComponent FAKE
    // msg_lineInfo[]  lines            #details of each line (from the detector) see the next message

    // Description msg_lineInfo (from the detector)
    // ## This message is part of msg_lines.
    // ## Since it does not have an header is not meant to be used stand alone
    // bool    isValid
    // int32   counter
    // float32 offset

    road_layout_estimation::msg_lines msg_lines;    ///< full message containing info

    ros::ServiceClient *getHighwayInfo_client;

    ros::Time       timestamp;

    double scoreLanes;
    double scoreWidth;
    double totalComponentScore;
    const double maxValueForGoodLine = 10;          ///< This parameter should reflect the isis-line-detector value

public:

    // Virtual functions -------------------------------------------------------
    void componentPoseEstimation(); //this is evoked by the chain Sampling > PropagateLayoutComponents > ComponentPoseEstimation ...
    void calculateComponentScore();
    void componentPerturbation();

    // Getters and setters -----------------------------------------------------
    double      getRoad_width()               const;
    double      getRoad_naiveWidth()          const;
    char        getCurrent_lane()             const;
    ros::Time   getTimestamp();
    void        setRoad_width                 (double value);
    void        setRoad_naiveWidth            (double value);
    void        setCurrent_lane               (char value);
    void        setTimestamp(ros::Time time);

    // Other public functions --------------------------------------------------
    int         getLanes_number()             const;///< Calculate the number of LANES given the number of lines stored inside msg_lines
    void        calculateCurrentLane();             ///< Calculate the current lane using the

    // Constructors and destructors --------------------------------------------
    LayoutComponent_RoadState()
    {
        particle_id = 0;
        component_id = 0;
        component_weight = 0;
        component_state = VectorXd::Zero(12);
        component_cov = MatrixXd::Zero(12, 12);
        //lanes_number = 0;
        //current_lane = 0;
        //road_width = 0.0f;
        //way_id = 0;
        timestamp = ros::Time(0);

        getHighwayInfo_client = NULL;
        oneway = 0;
    }

    /// This constructor should be used only in the initialization phase
    ROS_DEPRECATED LayoutComponent_RoadState(const unsigned int particle_id,
            const unsigned int component_id,
            int way_id,
            int lanes_number,
            const unsigned char current_lane,
            double road_width,
            ros::Time timestamp,
            ros::ServiceClient *serviceClientFromLayoutManager)
    {
        this->particle_id = particle_id;
        this->component_id = component_id;
        this->timestamp = timestamp;
        this->current_lane_wrt_OSM_distance = current_lane;

        //this->lanes_number= lanes_number;
        //this->road_width = road_width;
        //this->way_id=way_id;


        this->component_weight = 0;
        this->component_state = VectorXd::Zero(12);
        this->component_cov = MatrixXd::Zero(12, 12);

        getHighwayInfo_client = serviceClientFromLayoutManager;
    }

    /// This constructor should be used during the normal filter iteration
    LayoutComponent_RoadState(const unsigned int particle_id,
                              const unsigned int component_id,
                              ros::Time timestamp,
                              ros::ServiceClient *serviceClientFromLayoutManager,
                              const road_layout_estimation::msg_lines &msg_lines,
                              int32_t oneway)
    {
        this->particle_id   = particle_id;
        this->component_id  = component_id;
        this->timestamp     = timestamp;
        this->current_lane_wrt_OSM_distance  = 0;                // current lane is not set.
        this->msg_lines     = msg_lines;

        this->component_weight = 0;
        this->component_state = VectorXd::Zero(12);
        this->component_cov = MatrixXd::Zero(12, 12);

        this->oneway = oneway;

        getHighwayInfo_client = serviceClientFromLayoutManager;
    }

    ~LayoutComponent_RoadState()
    {
        particle_id = 0;
        component_id = 0;
        component_weight = 0;
        component_state.resize(0);
        component_cov.resize(0, 0);
        timestamp = ros::Time(0);
    }

    int64_t getWay_id() const;
    void    setWay_id(const int64_t &value);
    bool    getOneway() const;
    void    setOneway(const bool &value);
    double  getScoreLanes() const;
    void    setScoreLanes(double value);
    double  getScoreWidth() const;
    void    setScoreWidth(double value);
    double  getTotalComponentScore() const;
    void    setTotalComponentScore(double value);

    road_layout_estimation::msg_lines   getMsg_lines() const;
    void                                setMsg_lines(const road_layout_estimation::msg_lines &value);

};

#endif // LAYOUTCOMPONENT_ROADSTATE_H

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

    road_layout_estimation::msg_lines msg_lines;            ///< full message containing info

    ros::ServiceClient *getHighwayInfo_client;
    ira_open_street_map::getHighwayInfo getHighwayInfo;     ///< request+response from the service
    bool oneway;                                            ///< oneway, retrieved using getHighwayService service
    bool serviceOk;                                         ///< introduced with #536, used to check if the serviceCall was OK (stores the bool answer of the service call)

    ros::Time       timestamp;

    double scoreLanes;
    double scoreWidth;
    double totalComponentScore;
    const double maxValueForGoodLine = 10;                  ///< This parameter should reflect the isis-line-detector value
    double roadState_distribution_alpha;                    ///< the alpha once used inside the LayoutManager, after #534 inside each component

public:

    // Virtual functions -------------------------------------------------------
    void componentPoseEstimation(int index); //this is evoked by the chain Sampling > PropagateLayoutComponents > ComponentPoseEstimation ...
    void calculateComponentScore();
    void componentPerturbation();

    /**
     * @brief getAlphas
     * @return street_distribution_alpha + angle_distribution_alpha
     */
    double getAlphas();

    // Getters and setters -----------------------------------------------------
    double      getComponentRoad_width()               const;
    double      getComponentRoad_naiveWidth()          const;
    ros::Time   getTimestamp();
    void        setComponentRoad_width                 (double value);
    void        setComponentRoad_naiveWidth            (double value);
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

    LayoutComponent* clone()
    {
        LayoutComponent* cloned = new LayoutComponent_RoadState(*this);
        return cloned;
    }

    /// This constructor should be used only in the initialization phase
    ROS_DEPRECATED LayoutComponent_RoadState(const unsigned int particle_id,
                                             const unsigned int component_id,
                                             int way_id,
                                             int lanes_number,
                                             double road_width,
                                             ros::Time timestamp,
                                             ros::ServiceClient *serviceClientFromLayoutManager)
    {
        this->particle_id = particle_id;
        this->component_id = component_id;
        this->timestamp = timestamp;

        //this->lanes_number= lanes_number;
        //this->road_width = road_width;
        //this->way_id=way_id;


        this->component_weight = 0;
        this->component_state = VectorXd::Zero(12);
        this->component_cov = MatrixXd::Zero(12, 12);
        this->roadState_distribution_alpha = -10000; // #534 , since it is not used anymore i put an unfeseable value here.

        getHighwayInfo_client = serviceClientFromLayoutManager;
    }

    /// This constructor should be used during the normal filter iteration
    LayoutComponent_RoadState(const unsigned int particle_id,
                              const unsigned int component_id,
                              ros::Time timestamp,
                              ros::ServiceClient *serviceClientFromLayoutManager,
                              const road_layout_estimation::msg_lines &msg_lines,
                              int32_t oneway,
                              double roadState_distribution_alpha)
    {
        this->particle_id   = particle_id;
        this->component_id  = component_id;
        this->timestamp     = timestamp;
        this->msg_lines     = msg_lines;
        this->roadState_distribution_alpha = roadState_distribution_alpha;

        this->component_weight = 0;
        this->component_state = VectorXd::Zero(12);
        this->component_cov = MatrixXd::Zero(12, 12);

        this->oneway = oneway;

        ROS_ASSERT(this->roadState_distribution_alpha > 0.0f);

        getHighwayInfo_client = serviceClientFromLayoutManager;

        // this flag was introduced with #536, splittin the behavior calculateScore/componentPoseEstimation
        serviceOk = false; ///< default false, if the getHighwayInfo_client call is OK, then true, false again otherwise.
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

    double getRoadState_distribution_alpha() const;
    void setRoadState_distribution_alpha(double value);

    double  getOSMRoad_width() const;
};

#endif // LAYOUTCOMPONENT_ROADSTATE_H

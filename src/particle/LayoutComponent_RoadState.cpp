#include "LayoutComponent_RoadState.h"

/**
 * Implementation of pure virtual method 'calculateWeight'
 */

double LayoutComponent_RoadState::getRoad_width() const
{
    return msg_lines.width;
//    return road_width;
}

void LayoutComponent_RoadState::setRoad_width(double value)
{
    msg_lines.width=value;
//    road_width = value;
}

char LayoutComponent_RoadState::getCurrent_lane() const
{
    return current_lane;
}

void LayoutComponent_RoadState::setCurrent_lane(char value)
{
    current_lane = value;
}

int LayoutComponent_RoadState::getLanes_number() const
{
    return msg_lines.number_of_lines;
//    return lanes_number;
}

void LayoutComponent_RoadState::setLanes_number(int value)
{
    msg_lines.number_of_lines=value;
//    lanes_number = value;
}

int64_t LayoutComponent_RoadState::getWay_id() const
{
    return msg_lines.way_id;
//    return way_id;
}

void LayoutComponent_RoadState::setWay_id(const int64_t &value)
{
    msg_lines.way_id=value;
//    way_id = value;
}

///
/// \brief Implementation of pure virtual method LayoutComponent_RoadState::calculateComponentScore
/// using the component_state evaluate the OSM keys and give a score to the component
/// values to evaluate
///                   lanes_number (discrete) pdf > poisson
///                   road_with    (continue) pdf > gaussian
///
void LayoutComponent_RoadState::calculateComponentScore()
{
    ROS_DEBUG_STREAM("> Entering calculateComponentScore, component ID: " << component_id << " of particle ID: " << particle_id);

    ira_open_street_map::getHighwayInfo getHighwayInfo;
    getHighwayInfo.request.way_id = this->getWay_id();

    double scoreLanes = 1.0f;
    double scoreWidth = 1.0f;
    double totalComponentScore = 0.0f;

    if (getHighwayInfo_client->call(getHighwayInfo))
    {
        ROS_DEBUG_STREAM("calculateComponentScore, OSM says   wayId: " << this->getWay_id());
        ROS_DEBUG_STREAM("calculateComponentScore, OSM says   witdh: " << getHighwayInfo.response.width           << ", component says: " << this->getRoad_width()  );
        ROS_DEBUG_STREAM("calculateComponentScore, OSM says n#lanes: " << getHighwayInfo.response.number_of_lanes << ", component says: " << this->getLanes_number());

        boost::math::normal  normal_distribution(getHighwayInfo.response.width, 1.0f);     // Normal distribution.
        scoreWidth = pdf(normal_distribution, this->getRoad_width()) / pdf(normal_distribution, getHighwayInfo.response.width);
        ROS_DEBUG_STREAM("Width  Score (normalized-to-1 normal pdf): " << std::fixed << scoreWidth << "\t notNorm: " << pdf(normal_distribution, this->getRoad_width()));

        if (getHighwayInfo.response.number_of_lanes)
        {
            boost::math::poisson poisson_distribution(getHighwayInfo.response.number_of_lanes); // Poisson distribution: lambda/mean must be > 0
            scoreLanes = pdf(poisson_distribution,this->getLanes_number()) / pdf(poisson_distribution,getHighwayInfo.response.number_of_lanes);
            ROS_DEBUG_STREAM("Lanes  Score (normalzed-to-1 poisson pdf): " << std::fixed << scoreLanes << "\t notNorm: " <<  pdf(poisson_distribution,this->getLanes_number()));
        }

        char expected_lines = 0;
        if (this->msg_lines.number_of_lines>0)
        {
            expected_lines = this->msg_lines.number_of_lines + 1;
        }


        scoreWidth *= 1 - (this->msg_lines.goodLines / expected_lines);

        totalComponentScore = scoreLanes * scoreWidth;

        this->setComponentWeight(totalComponentScore);
        //have a look here: https://en.wikipedia.org/wiki/Scoring_rule

    }


    ROS_DEBUG_STREAM("< Exiting calculateComponentScore, component ID: " << component_id << " of particle ID: " <<particle_id);
}

/**
 * Implementation of pure virtual method 'componentPerturbation'
 *
 *      roadState:  road  width >> add little % noise
 *                  n# of lanes >> ??? leave unchanged
 */
void LayoutComponent_RoadState::componentPerturbation()
{
    ROS_INFO_STREAM_ONCE("componentPerturbation, component ID: " << component_id << " of particle ID: " << particle_id);

    //boost::mt19937 rng; // I don't seed it on purpouse (it's not relevant)
    //rng.seed(static_cast<unsigned int>(std::time(NULL) + getpid()));
    //boost::normal_distribution<> nd(1.0, 0.03333);
    //boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);
    //double road_width = this->getRoad_width() * var_nor();

    double road_width = this->getRoad_width() * Utils::box_muller(1,0.001);


    ROS_DEBUG_STREAM("roadWith: " << this->getRoad_width() << "\tPerturbed: " << road_width);

    this->setRoad_width(road_width);
}

int LayoutComponent_RoadState::linesFromLanes(int number_of_lanes)
{
    ROS_ASSERT(number_of_lanes>=0);

    if (number_of_lanes == 0)
        return 0;
    else
        return number_of_lanes + 1;

}

int LayoutComponent_RoadState::lanesFromLines(int goodLines)
{
    ROS_ASSERT(goodLines>=0);

    if ( (goodLines == 0) ||
         (goodLines == 1) ||
         (goodLines == 2)
       )
        return 1;
    else
        return (goodLines-1);
}

/**
 * Implementation of pure virtual method 'componentPoseEstimation'
 * In roadStateComponent this routine does the following, to 'predict' the new component 'state':
 *      1. given the particle position call snap_particle_xy, this return the way_id
 *      2. call getHighwayInfo(way_id)
 *      3. set new values to into the component
 *
 *      @now <<<do not update the values>>>
 */
void LayoutComponent_RoadState::componentPoseEstimation()
{
    ROS_DEBUG_STREAM("componentPoseEstimation, component ID: " << component_id << " of particle ID: " <<particle_id << " componentState: " << getComponentState()(0)<< ";" <<getComponentState()(1)<< ";" << getComponentState()(2));

    /*
     * il componente ha come <VectorXd component_state> lo stato (position) della particella .
     * come stimare il nuovo stato (è sbagliato componentPOSE dovrebbe essere componentSTATE estimation):
     *     this***          1- teniamo uguali quelli che ci sono
     *                      2- guardiamo cosa c'è nella strada più vicina dove l'ipotesi vive
     */

    // this measures are 'sensed', can't estimate

    //    this->lanes_number = this->lanes_number;
    //    this->road_width   = this->road_width;
    //    this->way_id       = this->way_id;

    this->setRoad_width     (this->getRoad_width());
    this->setLanes_number   (this->getLanes_number());
    this->setWay_id         (this->getWay_id());

}



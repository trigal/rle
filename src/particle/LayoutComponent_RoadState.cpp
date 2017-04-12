#include "LayoutComponent_RoadState.h"

double LayoutComponent_RoadState::getComponentRoad_width() const
{
    return state_width;
//    return road_width;
}

void LayoutComponent_RoadState::setComponentRoad_width(double value)
{
    state_width = value;
//    road_width = value;
}


/**
 * @brief LayoutComponent_RoadState::getLanes_number
 * @return the number of LANES given the number of lines.
 */
int LayoutComponent_RoadState::getLanes_number() const
{
    if (msg_lines.goodLines > 0)
        return Utils::lanesFromLines(msg_lines.goodLines);
    else
        return 0;// or ? Utils::lanesFromLines(msg_lines.number_of_lines);
}

void LayoutComponent_RoadState::setTimestamp(ros::Time time)
{
    timestamp = time;
}

ros::Time LayoutComponent_RoadState::getTimestamp()
{
    return timestamp;
}

int64_t LayoutComponent_RoadState::getWay_id() const
{
    return msg_lines.way_id;
    //    return way_id;
}

void LayoutComponent_RoadState::setWay_id(const int64_t &value)
{
    msg_lines.way_id = value;
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
    //ROS_WARN_STREAM("ROADSTATESCORE 1");

    //ira_open_street_map::getHighwayInfo getHighwayInfo;
    //getHighwayInfo.request.way_id = this->getWay_id();

    //if (getHighwayInfo_client->call(getHighwayInfo))
//#563    if (this->serviceOk)
    //{
    //ROS_DEBUG_STREAM("calculateComponentScore, OSM says   wayId: " << this->getWay_id());
    //ROS_DEBUG_STREAM("calculateComponentScore, OSM says   witdh: " << getHighwayInfo.response.width           << ", component says width:       " << this->getComponentRoad_width()      );
    //double OSMWidth = getHighwayInfo.response.width;
    double laneWidth = this->OSMWidth / this->number_of_lanes;

    //ROS_WARN_STREAM("OSMWidth: "<<OSMWidth<<", laneWidth: "<<laneWidth<<", n_lines: "<<getHighwayInfo.response.number_of_lanes);

    if (this->OSMWidth <= 0.)
    {
        this->scoreWidth = 0.0f;
        component_weight = 0.0f;
        return;
    }

    boost::math::normal mixture2(this->OSMWidth, 1);
    boost::math::normal mixture3(this->OSMWidth + laneWidth, 1);

    double weight_mixture1 = 0.2;
    double weight_mixture2 = 0.6;
    double weight_mixture3 = 0.2;
    double max_pdf, component_pdf;
    if (getHighwayInfo.response.number_of_lanes == 1)
    {
        weight_mixture1 = 0;
        weight_mixture2 = 0.8;
        weight_mixture3 = 0.2;

        max_pdf = weight_mixture2 * pdf(mixture2, this->OSMWidth) + weight_mixture3 * pdf(mixture3, this->OSMWidth);
        component_pdf = weight_mixture2 * pdf(mixture2, this->state_width) + weight_mixture3 * pdf(mixture3, this->state_width);
    }
    else
    {
        boost::math::normal mixture1(this->OSMWidth - laneWidth, 1);
        max_pdf = weight_mixture1 * pdf(mixture1, this->OSMWidth) + weight_mixture2 * pdf(mixture2, this->OSMWidth) + weight_mixture3 * pdf(mixture3, this->OSMWidth);
        component_pdf = weight_mixture1 * pdf(mixture1, this->state_width) + weight_mixture2 * pdf(mixture2, this->state_width) + weight_mixture3 * pdf(mixture3, this->state_width);
    }

    this->scoreWidth = component_pdf / max_pdf;
    component_weight = component_pdf / max_pdf;

    //ROS_ERROR_STREAM("TESTWIDTH - OSM-Width=" << OSMWidth << ", Component-Width=" << this->state_width <<", SCORE=" << this->scoreWidth);

    /*}
    else
    {
        ROS_ERROR_STREAM("Can't get HighwayInfo with wayId = " << this->getWay_id());
        this->scoreWidth          = 0.0f;
        component_weight          = 0.0f;
    }*/


    ROS_DEBUG_STREAM("< Exiting calculateComponentScore, component ID: " << component_id << " of particle ID: " << particle_id);
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

    //double road_width = this->getComponentRoad_width() * Utils::box_muller(1, 0.001);


    // ROS_DEBUG_STREAM("roadWith: " << this->getComponentRoad_width() << "\tPerturbed: " << road_width);

    //this->setComponentRoad_width(road_width);
}

double LayoutComponent_RoadState::getAlphas()
{
    return roadState_distribution_alpha;
}

double LayoutComponent_RoadState::getScoreWidth() const
{
    return scoreWidth;
}

void LayoutComponent_RoadState::setScoreWidth(double value)
{
    scoreWidth = value;
}

road_layout_estimation::msg_lines LayoutComponent_RoadState::getMsg_lines() const
{
    return msg_lines;
}

void LayoutComponent_RoadState::setMsg_lines(const road_layout_estimation::msg_lines &value)
{
    msg_lines = value;
}

/**
 * @brief LayoutComponent_RoadState::componentPoseEstimation Implementation of pure virtual method 'componentPoseEstimation'
 *
 * In roadStateComponent this routine does the following, to 'predict' the new component 'state':
 *      1. given the particle position call snap_particle_xy, this return the way_id
 *      2. call getHighwayInfo(way_id)
 *      3. set new values to into the component
 *
 *      @now <<<do not update the values>>> because ... read the following carefully
 *
 * Since this components stores the STATE of the road in which the particle lies
 * it contains also information regarding "current lane", "oneway" or "wayid"
 *
 * The component deleted/created every time a new "msg_lines" is received, this
 * is the reason because here nothing is really updated.
 *
 * The latter is performed inside <<LayoutManager::roadStateCallback>>
 *
 *
 * UPDATE 10/dec/2015, SPLITTING calculateComponentScore according to #536
 */
void LayoutComponent_RoadState::componentPoseEstimation(int index)
{
    ROS_DEBUG_STREAM("componentPoseEstimation, component ID: " << component_id << " of particle ID: " << particle_id << " componentState: " << getComponentState()(0) << ";" << getComponentState()(1) << ";" << getComponentState()(2));

    // carefully read the documentation to understand why these naive lines!
    this->setComponentRoad_width     (this->getComponentRoad_width());
    this->setWay_id         (this->getWay_id());
    this->setOneway         (this->getOneway());


    // #536
    getHighwayInfo.request.way_id = this->getWay_id();

    if (getHighwayInfo_client->call(getHighwayInfo))
    {
        /* I forget that the getHighwayInfo returns also if it is oneway, or just
         * this is happening after the split due to #536; this oneway is retreived
         * either here and during the creation of this component, that is re-created
         * inside the CALLBACK routine of the LayoutManager
         *
         * I'm gonna put this ASSERT, if stops here something is strange ...
         */


        this->OSMWidth = getHighwayInfo.response.width;
        this->number_of_lanes = getHighwayInfo.response.number_of_lanes;

        ROS_ASSERT_MSG(getHighwayInfo.response.oneway == this->getOneway(), "Check why the hell the oneway tags are different inside LayoutComponent_RoadState!");

        serviceOk = true;
    }
    else
    {
        ROS_ERROR_STREAM("Can't get HighwayInfo with wayId = " << this->getWay_id());

        serviceOk = false;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    double detector_width = 0, detector_weight = 0;

    if (msg_lines.width > 0)
    {
        detector_width = msg_lines.width;
        detector_weight = 0.9;
    }
    else if (msg_lines.naive_width > 0)
    {
        detector_width = msg_lines.naive_width * this->number_of_lanes;
        detector_weight = 0.7;

    }
    else
    {
        detector_width = 5;
        detector_weight = 0.1;
    }
    std::discrete_distribution<> d({(1 - detector_weight), detector_weight});

    if (this->state_width <= 0)
        this->state_width = 5.;

    std::array<std::normal_distribution<double>, 2> a =
    {
        std::normal_distribution<double>(this->state_width, 0.1),
        std::normal_distribution<double>(detector_width, 0.1)
    };

    int mixture_index;
    mixture_index = d(gen);
    double new_width = a[mixture_index](gen);

    //ROS_ERROR_STREAM("TESTSAMPLE - Old State=" << this->state_width << ", New State=" << new_width);

    this->state_width = new_width;


}


bool LayoutComponent_RoadState::getOneway() const
{
    return oneway;
}

void LayoutComponent_RoadState::setOneway(const bool &value)
{
    oneway = value;
}

double LayoutComponent_RoadState::getRoadState_distribution_alpha() const
{
    return roadState_distribution_alpha;
}

void LayoutComponent_RoadState::setRoadState_distribution_alpha(double value)
{
    roadState_distribution_alpha = value;
}

/**
 * @brief LayoutComponent_RoadState::getOSMRoad_width
 * @return returns the OSM road width, or -1 if the call to the highwayinfo service was not ok.
 */
double LayoutComponent_RoadState::getOSMRoad_width() const
{
    if (serviceOk)
        return getHighwayInfo.response.width;
    else
        return -1;
}

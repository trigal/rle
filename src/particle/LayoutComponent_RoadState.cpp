#include "LayoutComponent_RoadState.h"

double LayoutComponent_RoadState::getComponentRoad_width() const
{
    return msg_lines.width;
//    return road_width;
}

void LayoutComponent_RoadState::setComponentRoad_width(double value)
{
    msg_lines.width = value;
//    road_width = value;
}

double LayoutComponent_RoadState::getComponentRoad_naiveWidth() const
{
    return msg_lines.naive_width;
}

void LayoutComponent_RoadState::setComponentRoad_naiveWidth(double value)
{
    msg_lines.naive_width = value;
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

    ira_open_street_map::getHighwayInfo getHighwayInfo;
    getHighwayInfo.request.way_id = this->getWay_id();

    double scoreLanes            = 1.0f; // LINES to LANES conversion is in the getLanes_number() function
    double scoreWidth            = 1.0f;
    double scoreNaiveWidth       = 1.0f;
    double totalComponentScore   = 0.0f;

    double scaling_factor        = 0.0f; // calculated later in the code P(OSM | GOOD_DETECTION) = P (GOOD_DETECTION | OSM) * P (OSM) / P(GOOD_DETECTION)
    double OSM_lines_reliability = 1.0f;
    double detector_reliability  = 1.0f;

//#536    if (getHighwayInfo_client->call(getHighwayInfo))
    if (this->serviceOk)
    {
        ROS_DEBUG_STREAM("calculateComponentScore, OSM says   wayId: " << this->getWay_id());
        ROS_DEBUG_STREAM("calculateComponentScore, OSM says   witdh: " << getHighwayInfo.response.width           << ", component says width:       " << this->getComponentRoad_width()      );
        ROS_DEBUG_STREAM("calculateComponentScore, OSM says   witdh: " << getHighwayInfo.response.width           << ", component says naive_width: " << this->getComponentRoad_naiveWidth() );
        ROS_DEBUG_STREAM("calculateComponentScore, OSM says n#lanes: " << getHighwayInfo.response.number_of_lanes << ", component says:             " << this->getLanes_number()    );

        boost::math::normal  normal_distribution(getHighwayInfo.response.width, 1.0f);     // Normal distribution.
        scoreWidth      = pdf(normal_distribution, this->getComponentRoad_width())      / pdf(normal_distribution, getHighwayInfo.response.width);
        scoreNaiveWidth = pdf(normal_distribution, this->getComponentRoad_naiveWidth()) / pdf(normal_distribution, getHighwayInfo.response.width);
        ROS_DEBUG_STREAM("Width       Score (normalized-to-1 normal pdf): " << std::fixed << scoreWidth      << "\t not-Normalized: " << pdf(normal_distribution, this->getComponentRoad_width())      );
        ROS_DEBUG_STREAM("Naive Width Score (normalized-to-1 normal pdf): " << std::fixed << scoreNaiveWidth << "\t not-Normalized: " << pdf(normal_distribution, this->getComponentRoad_naiveWidth()) );

        /// 1. SCORE LANES
        /// If we have the number of lanes, update the scoreLanes value. Otherwise leave unchanged = 1.0f
        if (getHighwayInfo.response.number_of_lanes)
        {
            // Initialize the POISSON distribution
            boost::math::poisson poisson_distribution(getHighwayInfo.response.number_of_lanes); // Poisson distribution: lambda/mean must be > 0

            // Conversion from #LINES to #LANES inside getLanes_number
            scoreLanes = pdf(poisson_distribution, this->getLanes_number()) / pdf(poisson_distribution, getHighwayInfo.response.number_of_lanes);

            ROS_DEBUG_STREAM("Lanes  Score (normalzed-to-1 poisson pdf): " << std::fixed << scoreLanes << "\t not-Normalized: " <<  pdf(poisson_distribution, this->getLanes_number()));
        }

        ROS_DEBUG_STREAM ("GOOD LINES: " << this->msg_lines.goodLines << "\tALL LINES: " << this->msg_lines.number_of_lines);

        /// 2. SCORE WIDTH
        /// Calculate the scoreWith depending from the goodLines value.
        if (this->msg_lines.goodLines != 0)
        {
            // return the normalized scorewidth, scaled by a factor equal to the good lines tracked over the expected number of lines of OSM.
            scaling_factor = (this->msg_lines.goodLines / this->msg_lines.number_of_lines) * OSM_lines_reliability;

            double linesSum = 0.0f;
            for (int index = 0; index < this->msg_lines.goodLines; index++)
            {
                if (this->msg_lines.lines.at(index).isValid)
                {
                    linesSum += this->msg_lines.lines.at(index).counter;
                }
            }

            double linesLikelihood = 0.0f;
            linesLikelihood = linesSum / this->msg_lines.number_of_lines * this->maxValueForGoodLine;

            // TRY THIS. scaling_factor = linesLikelihood * OSM_lines_reliability / detector_reliability; [WARNING: linesLikelihood is linesSum now, wrong name before. It was and it is NOT used]

            ROS_ASSERT (scaling_factor <= 1.0f);
            ROS_DEBUG_STREAM("GoodLines > 0 [" << this->msg_lines.goodLines << "], using      Score " << scoreWidth << "\tScaling Factor: " << scaling_factor << " [goodL/numL,OSM_reliability]:" << (this->msg_lines.goodLines / this->msg_lines.number_of_lines)  << " " << OSM_lines_reliability);
            scoreWidth = scoreWidth * scaling_factor;
        }
        else
        {
            scaling_factor = 0.3f; //fixed, how much the width distance calculated using all the lines, not only the ones with the OK valid flag (tracked with n.steps)
            scoreWidth = scoreNaiveWidth * scaling_factor;
            ROS_DEBUG_STREAM("GoodLines = 0 [" << this->msg_lines.goodLines << "], using NaiveScore " << scoreWidth << "\tScaling Factor: " << scaling_factor );
        }

        // Normalize the totalComponentScore to sum up to one.
        totalComponentScore = roadState_distribution_alpha * ((scoreLanes + scoreWidth) / (2.0f - (1 - scaling_factor))); // refs #446 & #534 [alpha]

        ROS_DEBUG_STREAM("SCORE WIDTH: " << scoreWidth << "\tSCORE LANES: " << scoreLanes << "\tTOTAL SCORE: " << totalComponentScore);

        this->setComponentWeight(totalComponentScore);
        //have a look here: https://en.wikipedia.org/wiki/Scoring_rule

        // For publishing purposes - debug.
        this->totalComponentScore = totalComponentScore;
        this->scoreLanes          = scoreLanes;
        this->scoreWidth          = scoreWidth;

    }
    else
    {
        ROS_ERROR_STREAM("Can't get HighwayInfo with wayId = " << this->getWay_id());

        this->totalComponentScore = 0.0f;
        this->scoreLanes          = 0.0f;
        this->scoreWidth          = 0.0f;
    }


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

    double road_width = this->getComponentRoad_width() * Utils::box_muller(1, 0.001);


    ROS_DEBUG_STREAM("roadWith: " << this->getComponentRoad_width() << "\tPerturbed: " << road_width);

    this->setComponentRoad_width(road_width);
}

double LayoutComponent_RoadState::getAlphas()
{
    return roadState_distribution_alpha;
}



double LayoutComponent_RoadState::getScoreLanes() const
{
    return scoreLanes;
}

void LayoutComponent_RoadState::setScoreLanes(double value)
{
    scoreLanes = value;
}

double LayoutComponent_RoadState::getScoreWidth() const
{
    return scoreWidth;
}

void LayoutComponent_RoadState::setScoreWidth(double value)
{
    scoreWidth = value;
}

double LayoutComponent_RoadState::getTotalComponentScore() const
{
    return totalComponentScore;
}

void LayoutComponent_RoadState::setTotalComponentScore(double value)
{
    totalComponentScore = value;
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

        ROS_ASSERT_MSG(getHighwayInfo.response.oneway == this->getOneway(),"Check why the hell the oneway tags are different inside LayoutComponent_RoadState!");

        serviceOk=true;
    }
    else
    {
        ROS_ERROR_STREAM("Can't get HighwayInfo with wayId = " << this->getWay_id());

        serviceOk=false;
    }


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

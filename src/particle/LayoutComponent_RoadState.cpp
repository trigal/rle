#include "LayoutComponent_RoadState.h"

double LayoutComponent_RoadState::getRoad_width() const
{
    return msg_lines.width;
//    return road_width;
}

void LayoutComponent_RoadState::setRoad_width(double value)
{
    msg_lines.width = value;
//    road_width = value;
}

double LayoutComponent_RoadState::getRoad_naiveWidth() const
{
    return msg_lines.naive_width;
}

void LayoutComponent_RoadState::setRoad_naiveWidth(double value)
{
    msg_lines.naive_width = value;
}


char LayoutComponent_RoadState::getCurrent_lane() const
{
    return current_lane;
}

void LayoutComponent_RoadState::setCurrent_lane(char value)
{
    current_lane = value;
}

///
/// \brief LayoutComponent_RoadState::getLanes_number
/// \return the number of LANES given the number of lines.
///
int LayoutComponent_RoadState::getLanes_number() const
{
    if (msg_lines.goodLines > 0)
        return Utils::lanesFromLines(msg_lines.goodLines);
    else
        return 0;// or ? Utils::lanesFromLines(msg_lines.number_of_lines);
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

    if (getHighwayInfo_client->call(getHighwayInfo))
    {
        ROS_DEBUG_STREAM("calculateComponentScore, OSM says   wayId: " << this->getWay_id());
        ROS_DEBUG_STREAM("calculateComponentScore, OSM says   witdh: " << getHighwayInfo.response.width           << ", component says width:       " << this->getRoad_width()      );
        ROS_DEBUG_STREAM("calculateComponentScore, OSM says   witdh: " << getHighwayInfo.response.width           << ", component says naive_width: " << this->getRoad_naiveWidth() );
        ROS_DEBUG_STREAM("calculateComponentScore, OSM says n#lanes: " << getHighwayInfo.response.number_of_lanes << ", component says:             " << this->getLanes_number()    );

        boost::math::normal  normal_distribution(getHighwayInfo.response.width, 1.0f);     // Normal distribution.
        scoreWidth      = pdf(normal_distribution, this->getRoad_width())      / pdf(normal_distribution, getHighwayInfo.response.width);
        scoreNaiveWidth = pdf(normal_distribution, this->getRoad_naiveWidth()) / pdf(normal_distribution, getHighwayInfo.response.width);
        ROS_DEBUG_STREAM("Width       Score (normalized-to-1 normal pdf): " << std::fixed << scoreWidth      << "\t not-Normalized: " << pdf(normal_distribution, this->getRoad_width())      );
        ROS_DEBUG_STREAM("Naive Width Score (normalized-to-1 normal pdf): " << std::fixed << scoreNaiveWidth << "\t not-Normalized: " << pdf(normal_distribution, this->getRoad_naiveWidth()) );

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
        totalComponentScore = (scoreLanes + scoreWidth) / (2.0f - (1 - scaling_factor)); // refs #446

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

    double road_width = this->getRoad_width() * Utils::box_muller(1, 0.001);


    ROS_DEBUG_STREAM("roadWith: " << this->getRoad_width() << "\tPerturbed: " << road_width);

    this->setRoad_width(road_width);
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
 * @brief LayoutComponent_RoadState::componentPoseEstimation
 *
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
    ROS_DEBUG_STREAM("componentPoseEstimation, component ID: " << component_id << " of particle ID: " << particle_id << " componentState: " << getComponentState()(0) << ";" << getComponentState()(1) << ";" << getComponentState()(2));

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
    this->setWay_id         (this->getWay_id());
    //this->setLanes_number   (this->getLanes_number()); // deleted , see #446 for details

}


int32_t LayoutComponent_RoadState::getOneway() const
{
    return oneway;
}

void LayoutComponent_RoadState::setOneway(const int32_t &value)
{
    oneway = value;
}

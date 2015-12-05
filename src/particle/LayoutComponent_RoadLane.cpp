#include "LayoutComponent_RoadLane.h"
#include "Particle.h"

int LayoutComponent_RoadLane::getLanes() const
{
    return howManyLanes;
}

void LayoutComponent_RoadLane::setLanes(int value)
{
    howManyLanes = value;
}
bool LayoutComponent_RoadLane::myCompare(road_layout_estimation::msg_lineInfo a, road_layout_estimation::msg_lineInfo b)
{
    if (a.offset > b.offset)
        //if (std::fabs(a.offset) < std::fabs(b.offset))
        return true;
    else
        return false;
}


/**
 * @brief LayoutComponent_RoadLane::componentPoseEstimation
 *
 * STEP 1/3 of ComponentEstimation
 *
 * Disabled right now. Here we should update the detected lines using the
 * ego-motion and weighting again everything. With the ISIS lane detector
 * this is done inside the detector and I don't know how to change things
 * there.
 */
void LayoutComponent_RoadLane::componentPoseEstimation()
{
    ROS_DEBUG_STREAM ("Propagating and estimating ROAD LANE component pose. ID: " << component_id << " that belongs to particle ID: " << particle_id );
}


/**
 * @brief LayoutComponent_RoadLane::componentPerturbation
 *
 * STEP 2/3 of ComponentEstimation
 *
 * We can put some random noise inside the filtered values here.
 */
void LayoutComponent_RoadLane::componentPerturbation()
{
    ROS_DEBUG_STREAM ("Perturbating ROAD LANE component ID: " << component_id << " that belongs to particle ID: " << particle_id);
}


/**
 * @brief LayoutComponent_RoadLane::calculateComponentScore
 *
 * STEP 3/3 of ComponentEstimation (called from calculateLayoutComponentsWeight)
 *
 * Here calculate the score of the component, using the values of megavariabile
 * and some "logic". Finally updates the componentWeight (remember that it must
 * be in the 0..1 range)
 *
 * To retrieve the ONEWAY flag, use the LayoutComponent-particle pointer, #523
 *
 */
void LayoutComponent_RoadLane::calculateComponentScore()
{
    ROS_DEBUG_STREAM ("Calculating weight of ROAD LANE component ID: " << component_id << " that belongs to particle ID: " << particle_id);

    // this is "safe" since the RoadStateComponent is created every time a new msg_lines arrives.
    // with safe I mean it is updated almost frequently

    /// check if the associated particle has the TAG oneway; this is done asking
    /// to the parent particle to access the RoadLane component and retrieving
    /// the oneway flag using the getOneWayHelper function;
    bool isOneWay           = this->particle->getOneWayFlag();

    double roadWidth = 6.0f;                // rotal width of the OSM road
    double distanceFromWayCenter = -1.8f;   // distance from ROAD/Osm-Way center
    int currentLaneOSM = -1 ;                  ///< store the lane number here. -1 is not initialized. Let set 1 (one) as minimum, people count from 1 (strange thing..)

    Eigen::VectorXd  currentLaneStatusSummarized; //in this variable are summarized both probabilities of IsInLaneX+SensorOK and IsInLaneX+SensorBAD
    currentLaneStatusSummarized.resize(2);
    currentLaneStatusSummarized.setZero(2);
    for(int i = 0; i< megavariabile.rows() / 2; i++)
        currentLaneStatusSummarized(i) = megavariabile(i) + megavariabile(int(megavariabile.rows()/2)+i);

    // Testing phase here
    distanceFromWayCenter   = this->particle->distance_to_closest_segment;

    // check the returned flag
    if (isOneWay)
    {
        /// In this case we're in a road with ONEWAY tag active.
        /// Where is the 1st lane? It depends on the driving direction of course
        /// identified with the order of the nodes, eccept if we the opposite
        /// flag is rised

        // TODO: CHECK OPPOSITE FLAG. It should be sufficient to modify getOneWayFlag
        // again to int instead of boolean anche changing the sign
        // from +distanceFromWayCenter to -distanceFromWayCenter

        currentLaneOSM = int(((roadWidth) / 2.0f + distanceFromWayCenter) / standardLaneWidth) + 1;

        this->setComponentWeight(currentLaneStatusSummarized(currentLaneOSM));

    }
    else
    {
        //TODO: handle this possibility


    }


    ///////     // THIS IS RELATED WITH #502
    ///////     ira_open_street_map::getDistanceFromLaneCenter getDistanceFromLaneCenter_serviceMessage;
    ///////     getDistanceFromLaneCenter_serviceMessage.request.x = tf_pose_map_frame.getOrigin().getX();
    ///////     getDistanceFromLaneCenter_serviceMessage.request.y = tf_pose_map_frame.getOrigin().getY();
    ///////     //getDistanceFromLaneCenter_serviceMessage.request.way_id = (dynamic_cast<LayoutComponent_RoadState *>((*particle_itr).getLayoutComponents().at(0)))->getWay_id(); //FIX: find the right component (check also the cast)
    ///////     getDistanceFromLaneCenter_serviceMessage.request.way_id = (*particle_itr).getWayIDHelper(); // refs #502
    ///////     getDistanceFromLaneCenter_serviceMessage.request.current_lane = 0; //FIX: unused?!
    ///////
    ///////     if (LayoutManager::.call(getDistanceFromLaneCenter_serviceMessage))
    ///////     {
    ///////         //ROS_ERROR_STREAM ("STICA!   " << getDistanceFromLaneCenter_serviceMessage.request.way_id << "\t " << getDistanceFromLaneCenter_serviceMessage.response.distance_from_lane_center << "\t" << getDistanceFromLaneCenter_serviceMessage.response.distance_from_way_center);
    ///////     }


    //need to sum up to ONE;
    this->setComponentWeight(0);
}


/**
 * @brief LayoutComponent_RoadLane::filter
 * @param msg_lines
 *
 * Updates the Hidden Markov Model
 * In the future, convert HMM to DBN will be required.
 * This routine is related to #519, follows #469
 *
 * It is called from the roadLaneCallback (inside the LayoutManager)
 */
void LayoutComponent_RoadLane::filter(const road_layout_estimation::msg_lines & msg_lines)
{
    cout << "================================================================" << endl << endl;
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " -- ComponentID/ParticleID: " << this->component_id << "/" << this->particle_id);
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    cout << megavariabile.sum() << endl;
    ROS_ASSERT( (1 - megavariabile.sum()) <= 0.00001f);

    // from excel, unified-transition matrix with 2 lanes + broken-sensor
    //Eigen::MatrixXd stateTransitionMatrix(4, 4);
    //stateTransitionMatrix << 0.693  , 0.297 , 0.007 , 0.003,
    //                      0.297  , 0.693 , 0.003 , 0.007,
    //                      0.07   , 0.03  , 0.63  , 0.27 ,
    //                      0.03   , 0.07  , 0.27  , 0.63 ;
    cout << "Transition Matrix" << endl;
    cout << stateTransitionMatrix.format(CleanFmt) << endl;

    //    cout << megavariabile.transpose() << " * " << state.col(0).array().transpose() << endl; //checking column/row multiplcation...

    Eigen::ArrayXd prediction(4);
    for (int i = 0; i < 4; i++)
        prediction[i] = (megavariabile * stateTransitionMatrix.col(i).array()).sum();

    unsigned int hypothesisCurrentLanesNaive = 0.0f; //hypothesis of current lane
    unsigned int hypothesisCurrentLanes = 0.0f; //hypothesis of current lane

    hypothesisCurrentLanesNaive = round(msg_lines.naive_width / standardLaneWidth );
    hypothesisCurrentLanes =      round(msg_lines.width / standardLaneWidth );

    /// Sorting lines
    vector <road_layout_estimation::msg_lineInfo> validAndSorted;
    for (int i = 0; i < msg_lines.lines.size(); ++i)
    {
        road_layout_estimation::msg_lineInfo toadd = msg_lines.lines.at(i);
        if (toadd.isValid)
            validAndSorted.push_back(toadd);
    }

    std::sort (validAndSorted.begin(), validAndSorted.end(), boost::bind(&LayoutComponent_RoadLane::myCompare, this, _1, _2));

    int corsie = getLanes() ; // qui osm FROM THE PARTICLE, need to be updated! related task #520
    int extra_corsie = corsie - hypothesisCurrentLanes; // quante altre penso che ci siano oltre a quella trovata OSM - TROVATE
    if (extra_corsie < 0)
        extra_corsie = 0; //non può essere minore di zero. caso in cui ho più detection di quelle che dovrebbero esserci

    Eigen::ArrayXd tentative(corsie); // il numero di corsie ipotizzate = OSM
    tentative.setZero(corsie);

    // if the detected with (not naive, with VALID lines) ...
    if (msg_lines.width > standardLaneWidth)
    {
        // se ci sono almeno due linee, quindi almeno una corsia
        if (msg_lines.goodLines > 1)
        {
            /// msg_lines.width is grater than standardLaneWidth and we have
            /// more than 1 good-tracked-line. Using the offsets, we can create
            /// some guesses

            while (extra_corsie + 1)
            {
                /// In quale delle corsie su hypothesisCurrentLanes sono?
                int toadd = corsie;
                for (int i = 0; i < msg_lines.goodLines; i++)
                {
                    if (std::fabs(validAndSorted[i].offset) < standardLaneWidth)
                    {
                        if (toadd == 0)
                            break;
                        tentative(toadd - 1) = tentative(toadd - 1) + 1;
                        //ROS_ASSERT ((toadd-1)>=1); //Stop if some value in the array is > 1, at the moment no-value should ..
                        break;
                    }
                    else
                        toadd--;
                }
                corsie--;
                extra_corsie--;
            }

            /// Does some couple of lines create a plausible "lane?"
            for (int i = 0; i < validAndSorted.size() - 1; i++)
            {
                double sum = (std::fabs(validAndSorted.at(i).offset) + std::fabs(validAndSorted.at(i + 1).offset));
                double threshold = standardLaneWidth / 2.0f;
                if ((validAndSorted.at(i).offset > 0.0f) && (validAndSorted.at(i + 1).offset < 0.0f))
                    if ( (sum > (standardLaneWidth - threshold)) &&
                            (sum < (standardLaneWidth + threshold)) )
                        ROS_INFO_STREAM("Inside a lane created by lines " << i << " and " << i + 1 << " " << sum);
                //ROS_INFO_STREAM("Inside a lane created by lines " << i << " and " << i + 1 << " " << sum);
            }

        }
        else
        {
            /// msg_lines.width is grater than standardLaneWidth, meaning that
            /// the only detected line should not be part of the "current lane".
            /// Thus, we can say that we're somewhere else but not in that lane

            ROS_WARN_STREAM("Not enough goodLines: " << msg_lines.goodLines << " - width: " << msg_lines.width );

            // standardLaneWidth > Threshold BUT goodLines = 1 AKA
            // only one line away from me more than 1 LANE threshold
            // I only can say that

            //sensor << 0.25, 0.25, 0.25, 0.25 ;
            if (validAndSorted.size() == 1)
            {
                // The following two IF could be merged, but in this way it is "pretty" clear =)
                int corsie = hypothesisCurrentLanes;

                if (validAndSorted.at(0).offset > 0)
                {
                    // adding ONES to all LEFT lanes compatible with the ONELINE offset
                    // i.e. all lines farther than validAndSorted, with the value on the RIGHT
                    double tot = standardLaneWidth;
                    while (corsie >= 0)
                    {
                        if (validAndSorted.at(0).offset < (tot))
                            tentative(corsie) = 1;

                        tot += standardLaneWidth;
                        corsie--;
                    }
                }
                else
                {
                    // adding ONES to all RIGHT lanes compatible with the ONELINE offset
                    // i.e. all lines farther than validAndSorted, with the value on the LEFT
                    double tot = standardLaneWidth;
                    while (corsie >= 0)
                    {
                        if (std::fabs(validAndSorted.at(0).offset) < (tot))
                            tentative(hypothesisCurrentLanes - corsie) = 1;

                        tot += standardLaneWidth;
                        corsie--;
                    }
                }

                /// Add noise to every guess (moved outside the if)
                //tentative += 0.001f;
                //tentative /= tentative.sum();
            }
            else
                ROS_ASSERT_MSG(1, "nope ... ");
        }
    }
    else
    {
        /// msg_lines.width is less than standardLaneWidth. We're close to the
        /// only one detected line, so I can't say nothing at all.

        tentative.setConstant(corsie, 1.0f / corsie);
        //ROS_WARN_STREAM(tentative[0] << " " << tentative[1] << " " << tentative[2]);
        string s;
        for (int i = 0; i <= tentative.cols(); i++)
            s = s + " " + std::to_string(tentative[i]);
        ROS_WARN_STREAM(s);
        //ROS_WARN_STREAM(tentative[0] << " " << tentative[1] << " ** Detected width less than standardLaneWidth (" << standardLaneWidth << "): " << msg_lines.width);
        //ROS_WARN_STREAM("Detected width less than standardLaneWidth: " << msg_lines.width);
    }

    /// Add noise to every guess and normalize
    tentative += 0.001f;
    tentative /= tentative.sum();

    /// After all this mess, I may have put valid lines inside some LANE that is
    /// is the opposite driving direction.

    // Sums the counter inside VALID lines.
    int counter = 0;
    for (int i = 0; i < msg_lines.lines.size(); i++)
        if (msg_lines.lines.at(i).isValid)
            counter += msg_lines.lines.at(i).counter;

    double SensorOK  = 0.0f; //double(counter) / double(getLanes() * MAX_CLOTHOID_R);
    double SensorBAD = 0.0f; //1-SensorOK;
    SensorOK  = double(counter) / double(getLanes() * MAX_COUNT) - 0.01f; //adding noise
    if (SensorOK < 0.0f)
        SensorOK = 0.01;
    SensorBAD = double(1.0f) - SensorOK;

    ROS_INFO_STREAM("Sensor OK/BAD:" << SensorOK << "/" << SensorBAD << " - counter: " << counter << "/" << getLanes() * MAX_COUNT);

    Eigen::Array2d a;
    Eigen::Array2d b;
    a = (tentative * SensorOK ) ; //<< endl;
    b = (tentative * SensorBAD) ; //<< endl;
    sensor.setZero(4);
    sensor << a, b;

    cout << "sensor     :\t" << sensor.transpose().format(CleanFmt) << endl;
    cout << "prediction :\t" << prediction.transpose().format(CleanFmt) << endl;
    Eigen::Vector4d update;
    update << sensor.array() * prediction.array() ;
    //cout << "no-norm-upd:\t" << update.transpose().format(CleanFmt) << endl;
    double sum = update.sum();
    //cout << "norm. fact :\t" << sum << endl;
    update /= sum;
    cout << "update     :\t" << update.transpose().format(CleanFmt) << endl;
    cout << "summarize  :\t" << update(0) + update(2) << " | " << update(1) + update(3) << endl;

    ROS_ASSERT( (1 - update.sum()) <= 0.00001f);

    megavariabile = update;

}

void LayoutComponent_RoadLane::resetSensor(int lanes)
{
    ROS_ASSERT_MSG(lanes >= 0, "lanes must be greater than zero");

    if (lanes == 0)
        sensor.resize(getLanes());
    else
        sensor.resize(lanes);
}

void LayoutComponent_RoadLane::resetMegavariabile(int lanes)
{
    ROS_ASSERT_MSG(lanes >= 0, "lanes must be greater than zero");

    if (lanes == 0)
    {
        megavariabile.resize(getLanes() * 2);
        megavariabile.setConstant(1.0f / double(getLanes() * 2));
    }
    else
    {
        megavariabile.resize(lanes * 2);
        megavariabile.setConstant(1.0f / double(lanes) * 2);
    }

}

void LayoutComponent_RoadLane::setStateTransitionMatrix()
{
    stateTransitionMatrix.resize(4, 4);
    stateTransitionMatrix << 0.693  , 0.297 , 0.007 , 0.003,
                          0.297  , 0.693 , 0.003 , 0.007,
                          0.07   , 0.03  , 0.63  , 0.27 ,
                          0.03   , 0.07  , 0.27  , 0.63 ;
}

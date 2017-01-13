#include <string>
#include "ros/ros.h"
#include "road_layout_estimation/msg_lines.h"
#include "road_layout_estimation/msg_lineInfo.h"
#include <limits>
#include <algorithm>    // std::sort
#include <vector>       // std::vector

#include <eigen3/Eigen/Core>

using namespace std;

Eigen::VectorXd sensor;
Eigen::ArrayXd megavariabile;
Eigen::MatrixXd stateTransitionMatrix;
bool first = true;

int howManyLanes;                   ///< The number of lanes in the current hypothesis
double standardLaneWidth = 5.0f;    // maybe minLaneWidth
int MAX_COUNT = 10;

bool myCompare(road_layout_estimation::msg_lineInfo a, road_layout_estimation::msg_lineInfo b)
{
    if (a.offset > b.offset)
        //if (std::fabs(a.offset) < std::fabs(b.offset))
        return true;
    else
        return false;
}

bool isFalse(road_layout_estimation::msg_lineInfo a)
{
    if (!a.isValid)
        return true;
    else
        return false;
}

int lanesFromLines(int goodLines)
{
    ROS_ASSERT(goodLines >= 0);

    if ( (goodLines == 0) ||
            (goodLines == 1) ||
            (goodLines == 2)
       )
        return 1;
    else
        return (goodLines - 1);
}

void setStateTransitionMatrix()
{
    // have a look here #520
    if (howManyLanes == 2)
    {
        stateTransitionMatrix.resize(4, 4);
        stateTransitionMatrix << 0.693  , 0.297 , 0.007 , 0.003,
                              0.297  , 0.693 , 0.003 , 0.007,
                              0.07   , 0.03  , 0.63  , 0.27 ,
                              0.03   , 0.07  , 0.27  , 0.63 ;
    }
    else if (howManyLanes == 3)
    {
        stateTransitionMatrix.resize(6, 6);
        stateTransitionMatrix << 0.63,  0.269,  0.091,  0.006,  0.003,  0.001,
                              0.228, 0.534,  0.228,  0.002,  0.006,  0.002,
                              0.091, 0.269,  0.63,   0.001,  0.003,  0.006,
                              0.063, 0.027,  0.009,  0.567,  0.243,  0.091,
                              0.023, 0.054,  0.023,  0.208,  0.484,  0.208,
                              0.009, 0.027,  0.063,  0.091,  0.243,  0.567;


    }
}

void resetMegavariabile(int lanes)
{
    ROS_ASSERT_MSG(lanes >= 0, "lanes must be greater than zero");

    if (lanes == 0)
    {
        megavariabile.resize(howManyLanes * 2);
        megavariabile.setConstant(1.0f / double(howManyLanes * 2));
    }
    else
    {
        megavariabile.resize(lanes * 2);
        megavariabile.setConstant(1.0f / double(lanes * 2));
    }

}

void resetSensor(int lanes)
{
    ROS_ASSERT_MSG(lanes >= 0, "lanes must be greater than zero");

    if (lanes == 0)
        sensor.resize(howManyLanes);
    else
        sensor.resize(lanes);
}

/**
 * @brief chatterCallback
 * @param msg_lines
 *
 * this should became the roadLaneCallback
 */
void chatterCallback(const road_layout_estimation::msg_lines & msg_lines)
{
    cout << "================================================================" << endl << endl;
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    cout << megavariabile.sum() << endl;
    ROS_ASSERT( (1 - megavariabile.sum()) <= 0.00001f);

    // from excel, unified-transition matrix with 2 lanes + broken-sensor
    //Eigen::MatrixXd stateTransitionMatrix(4, 4);
    //stateTransitionMatrix << 0.693  , 0.297 , 0.007 , 0.003,
    //                      0.297  , 0.693 , 0.003 , 0.007,
    //                      0.07   , 0.03  , 0.63  , 0.27 ,
    //                      0.03   , 0.07  , 0.27  , 0.63 ;
    /*cout << "Transition Matrix" << endl;
    cout << stateTransitionMatrix.format(CleanFmt) << endl;
    cout << "Mega Variabile" << endl;
    cout << megavariabile.format(CleanFmt) << endl;*/

    //    cout << megavariabile.transpose() << " * " << state.col(0).array().transpose() << endl; //checking column/row multiplcation...

    Eigen::ArrayXd prediction(howManyLanes * 2); // have a look here #520
    for (int i = 0; i < howManyLanes * 2; i++)
        prediction[i] = (megavariabile * stateTransitionMatrix.col(i).array()).sum();

    //unsigned int hypothesisCurrentLanesNaive = 0.0f; //hypothesis of current lane
    unsigned int hypothesisCurrentLanes = 0.0f; //hypothesis of current lane

    //hypothesisCurrentLanesNaive = round(msg_lines.naive_width / standardLaneWidth );
    if (msg_lines.width > 0)
        hypothesisCurrentLanes =      round(msg_lines.width / standardLaneWidth );
    else
        hypothesisCurrentLanes = howManyLanes;

    /// Sorting lines
    vector <road_layout_estimation::msg_lineInfo> validAndSorted;
    for (int i = 0; i < msg_lines.lines.size(); ++i)
    {
        road_layout_estimation::msg_lineInfo toadd = msg_lines.lines.at(i);
        if (toadd.isValid)
            validAndSorted.push_back(toadd);
    }

    std::sort (validAndSorted.begin(), validAndSorted.end(), myCompare);

    int corsie = howManyLanes ; // qui osm FROM THE PARTICLE, need to be updated! related task #520
    int extra_corsie = corsie - hypothesisCurrentLanes; // quante altre penso che ci siano oltre a quella trovata OSM - TROVATE
    if (extra_corsie < 0)
        extra_corsie = 0; //non può essere minore di zero. caso in cui ho più detection di quelle che dovrebbero esserci

    Eigen::ArrayXd tentative(corsie); // il numero di corsie ipotizzate = OSM
    tentative.setZero(corsie);

    // if the detected with (not naive, with VALID lines) ...
    //if (msg_lines.width > standardLaneWidth)
    //{
    // se ci sono almeno due linee, quindi almeno una corsia
    double width = 0;
    if (validAndSorted.size() > 0)
        width = validAndSorted[0].offset - validAndSorted[msg_lines.goodLines - 1].offset;
    if (width > standardLaneWidth)
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
                    if (toadd <= 0)
                        break;
                    tentative(toadd - 1) = tentative(toadd - 1) + 1;
                    //ROS_ASSERT \((toadd-1)>=1); //Stop if some value in the array is > 1, at the moment no-value should ..
                    break;
                }
                else
                    toadd--;
            }
            corsie--;
            extra_corsie--;
        }
        cout << "Tentative: \t" << tentative(0) << " | " << tentative(1) << " | " << tentative(2) << endl;

        /// Does some couple of lines create a plausible "lane?"
        /*for (int i = 0; i < validAndSorted.size() - 1; i++)
        {
            double sum = (std::fabs(validAndSorted.at(i).offset) + std::fabs(validAndSorted.at(i + 1).offset));
            double threshold = standardLaneWidth / 2.0f;
            if ((validAndSorted.at(i).offset > 0.0f) && (validAndSorted.at(i + 1).offset < 0.0f))
                if ( (sum > (standardLaneWidth - threshold)) &&
                        (sum < (standardLaneWidth + threshold)) )
                    ROS_INFO_STREAM("Inside a lane created by lines " << i << " and " << i + 1 << " " << sum);
            //ROS_INFO_STREAM("Inside a lane created by lines " << i << " and " << i + 1 << " " << sum);
        }*/

    }
    else if (msg_lines.goodLines == 1)
    {
        if (validAndSorted.at(0).offset > standardLaneWidth || validAndSorted.at(0).offset < -standardLaneWidth)
        {
            /// msg_lines.width is grater than standardLaneWidth, meaning that
            /// the only detected line should not be part of the "current lane".
            /// Thus, we can say that we're somewhere else but not in that lane

            ROS_WARN_STREAM("Not enough goodLines: " << msg_lines.goodLines << " - width: " << msg_lines.width );

            // standardLaneWidth > Threshold BUT goodLines = 1 AKA
            // only one line away from me more than 1 LANE threshold
            // I only can say that

            //sensor << 0.25, 0.25, 0.25, 0.25 ;
            // The following two IF could be merged, but in this way it is "pretty" clear =)
            int corsie = hypothesisCurrentLanes - 1;

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
                        tentative(hypothesisCurrentLanes - corsie - 1) = 1;

                    tot += standardLaneWidth;
                    corsie--;
                }
            }

            /// Add noise to every guess (moved outside the if)
            //tentative += 0.001f;
            //tentative /= tentative.sum();
        }

        else if (validAndSorted.at(0).continuous == true)
        {
            if (validAndSorted.at(0).offset < 0)
            {
                tentative(0)++;
            }
            else
                tentative(howManyLanes - 1)++;
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
    SensorOK  = double(counter) / double((howManyLanes + 1) * MAX_COUNT) - 0.01f; //adding noise
    if (SensorOK < 0.0f)
        SensorOK = 0.01;
    if (SensorOK >= 1.0f)
        SensorOK = 0.99;
    SensorBAD = double(1.0f) - SensorOK;

    ROS_INFO_STREAM("Sensor OK/BAD:" << SensorOK << "/" << SensorBAD << " - counter: " << counter << "/" << (howManyLanes + 1) * MAX_COUNT);


    ///AAAAA TEST
    /*if (first)  {
        tentative(0) = 0.9;
        SensorOK = 0.99;
        first = false;
    }
    else {
        tentative(0) = 0.5;
        SensorOK = 0.1;
    }
    tentative(1) = 1.0 - tentative(0);
    SensorBAD = 1.0 - SensorOK;*/
    ///AAAAA

    Eigen::ArrayXd a;
    Eigen::ArrayXd b;
    Eigen::ArrayXd sensor_bad_state;
    a.setZero(howManyLanes);
    b.setZero(howManyLanes);
    sensor_bad_state.setZero(howManyLanes);
    double weight = 0.4;
    double p1, p2, p3;
    //p1 = (megavariabile(0) + megavariabile(2)) * weight;
    //p1 += 0.5 * (1 - weight);
    p1 = tentative(0)  * weight;
    p1 +=  (megavariabile(0) + megavariabile(howManyLanes)) * (1 - weight);
    if (howManyLanes == 2)
    {
        sensor_bad_state << p1, (1 - p1);
    }
    else
    {
        p2 = tentative(1)  * weight;
        p2 +=  (megavariabile(1) + megavariabile(howManyLanes + 1)) * (1 - weight);
        p3 = 1.0 - p1 - p2;
        sensor_bad_state << p1, p2, p3;
    }
    //sensor_bad_state << 0.5, 0.5;
    a = (tentative * SensorOK ) ; //<< endl;
    b = (sensor_bad_state * SensorBAD) ; //<< endl;
    sensor.setZero(howManyLanes * 2);
    sensor << a, b;

    cout << "SensorState:\t" << sensor_bad_state << endl;
    cout << "Tentative  :\t" << tentative << endl;
    cout << "sensor     :\t" << sensor.transpose().format(CleanFmt) << endl;
    cout << "prediction :\t" << prediction.transpose().format(CleanFmt) << endl;
    Eigen::VectorXd update;
    update.setZero(howManyLanes * 2);
    update << sensor.array() * prediction.array() ;
    //cout << "no-norm-upd:\t" << update.transpose().format(CleanFmt) << endl;
    double sum = update.sum();
    //cout << "norm. fact :\t" << sum << endl;
    update /= sum;
    cout << "update     :\t" << update.transpose().format(CleanFmt) << endl;
    if (howManyLanes == 2)
        cout << "summarize  :\t" << update(0) + update(2) << " | " << update(1) + update(3) << endl;
    else
        cout << "summarize  :\t" << update(0) + update(3) << " | " << update(1) + update(4) << " | " << update(2) + update(5) << endl;

    ROS_ASSERT( (1 - update.sum()) <= 0.00001f);

    megavariabile = update;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");

    /// This sets the logger level; use this to disable all ROS prints
    if ( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
        ros::console::notifyLoggerLevelsChanged();
    else
        std::cout << "Error while setting the logger level!" << std::endl;

    ros::NodeHandle n;

    howManyLanes = 2;
    //megavariabile.resize(4);
    //megavariabile.setConstant(1.0f / 4.0f);
    setStateTransitionMatrix();
    resetMegavariabile(howManyLanes);
    resetSensor(howManyLanes);

    ros::Subscriber sub = n.subscribe("/isis_line_detector/lines", 1, chatterCallback);

    ros::spin();

    return 0;
}
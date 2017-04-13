#include <string>
#include "ros/ros.h"
#include "road_layout_estimation/msg_lines.h"
#include "road_layout_estimation/msg_lineInfo.h"
#include <limits>
#include <algorithm>    // std::sort
#include <vector>       // std::vector

#include <eigen3/Eigen/Core>

#include <fstream>
#include <std_msgs/Bool.h>

using namespace std;

Eigen::VectorXd sensor;
Eigen::ArrayXd megavariabile;
Eigen::MatrixXd stateTransitionMatrix;
bool first = true;

int howManyLanes;                   ///< The number of lanes in the current hypothesis
double standardLaneWidth = 3.5f;    // maybe minLaneWidth
int MAX_COUNT = 10;

ros::Publisher *chatter_pub ;

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
        stateTransitionMatrix << 0.63,  0.279,  0.001,  0.0869,  0.003,  0.0001,
                              0.139, 0.63,  0.139,  0.003,  0.086,  0.003,
                              0.001, 0.279,  0.63,   0.0001,  0.003,  0.0869,
                              0.1519, 0.027,  0.0001,  0.567,  0.253,  0.001,
                              0.023, 0.054,  0.023,  0.208,  0.484,  0.208,
                              0.0001, 0.027,  0.1519,  0.001,  0.253,  0.567;


    }
    else if (howManyLanes == 4)
    {
        stateTransitionMatrix.resize(8, 8);
        stateTransitionMatrix << 0.641734,  0.244611,   0.013547,   0.000109,   0.071304,   0.027179,   0.001505,   1.21114E-05,
                              0.192354,   0.504639,   0.192354,   0.010653,   0.021373,   0.056071,   0.021373,   0.00118364,
                              0.010653,   0.192354,   0.504639,   0.192354,   0.001184,   0.021373,   0.056071,   0.021372669,
                              0.000109,   0.013547,   0.244611,   0.641734,   1.21E-05,   0.001505,   0.027179,   0.071303744,
                              0.128347,   0.048922,   0.002709,   2.18E-05,   0.513387,   0.195688,   0.010837,   8.72022E-05,
                              0.038471,   0.100928,   0.038471,   0.002131,   0.153883,   0.403711,   0.153883,   0.008522208,
                              0.002131,   0.038471,   0.100928,   0.038471,   0.008522,   0.153883,   0.403711,   0.153883217,
                              2.18E-05,   0.002709,   0.048922,   0.128347,   8.72E-05,   0.010837,   0.195688,   0.513386957;

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

    cout << msg_lines.header.seq << endl;
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
    //if (msg_lines.width > 0)
    //    hypothesisCurrentLanes =      round(msg_lines.width / standardLaneWidth );
    //else
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

    for (int i = 0; i < msg_lines.goodLines; i++)
    {
        if (validAndSorted.at(i).offset > standardLaneWidth || validAndSorted.at(i).offset < -standardLaneWidth)
        {

            // standardLaneWidth > Threshold BUT goodLines = 1 AKA
            // only one line away from me more than 1 LANE threshold
            // I only can say that

            //sensor << 0.25, 0.25, 0.25, 0.25 ;
            // The following two IF could be merged, but in this way it is "pretty" clear =)
            int corsie = hypothesisCurrentLanes - 1;

            if (validAndSorted.at(i).offset > 0)
            {
                // adding ONES to all LEFT lanes compatible with the ONELINE offset
                // i.e. all lines farther than validAndSorted, with the value on the RIGHT
                double tot = standardLaneWidth;
                while (corsie >= 0)
                {
                    if (validAndSorted.at(i).offset < (tot))
                        tentative(corsie)++;

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
                    if (std::fabs(validAndSorted.at(i).offset) < (tot))
                        tentative(hypothesisCurrentLanes - corsie - 1)++;

                    tot += standardLaneWidth;
                    corsie--;
                }
            }

            if (validAndSorted.at(i).continuous == true)
            {
                int corsia_attuale = validAndSorted.at(i).offset / standardLaneWidth;
                if (corsia_attuale > 0)
                    corsia_attuale = howManyLanes - corsia_attuale - 1;
                else if (corsia_attuale < 0)
                    corsia_attuale = -corsia_attuale;

                if (corsia_attuale >= 0 && corsia_attuale < howManyLanes)
                    tentative(corsia_attuale) += 1;
            }

            /// Add noise to every guess (moved outside the if)
            //tentative += 0.001f;
            //tentative /= tentative.sum();
        }
        else if (validAndSorted.at(i).continuous == true)
        {
            if (validAndSorted.at(i).offset < 0)
            {
                tentative(0)+=1;
            }
            else
                tentative(howManyLanes - 1)+=1;
        }
    }
    if (tentative.sum() <= 0)
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
    double weight = 0.6;
    /*double p1, p2, p3, p4;
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
    }*/
    double p[howManyLanes];
    for (int i = 0; i < howManyLanes; i++)
    {
        p[i] = tentative(i)  * weight;
        p[i] +=  (megavariabile(i) + megavariabile(howManyLanes + i)) * (1 - weight);
        sensor_bad_state(i) = p[i];
    }
    //sensor_bad_state << 0.5, 0.5;
    a = (tentative * SensorOK ) ; //<< endl;
    b = (sensor_bad_state * SensorBAD) ; //<< endl;
    sensor.setZero(howManyLanes * 2);
    sensor << a, b;

    //cout << "SensorState:\t" << sensor_bad_state << endl;
    cout << "Tentative  :\t" << tentative.transpose().format(CleanFmt) << endl;
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
    else if (howManyLanes == 3)
        cout << "summarize  :\t" << update(0) + update(3) << " | " << update(1) + update(4) << " | " << update(2) + update(5) << endl;
    else if (howManyLanes == 4)
        cout << "summarize  :\t" << update(0) + update(4) << " | " << update(1) + update(5) << " | " << update(2) + update(6) << " | " << update(3) + update(7) << endl;

    ROS_ASSERT( (1 - update.sum()) <= 0.00001f);

    megavariabile = update;

    ///STATS
    ofstream myfile;
    myfile.open ("/home/catta/lane.txt", ios::app);
    myfile << msg_lines.way_id  << ";" << SensorOK << ";" << SensorBAD << ";"
           << tentative(0) << ";" << tentative(1) << ";" << tentative(2) << ";" << tentative(3) << ";"
           << sensor(0) << ";" << sensor(1) << ";" << sensor(2) << ";" << sensor(3) << ";"
           << sensor(4) << ";" << sensor(5) << ";" << sensor(6) << ";" << sensor(7) << ";"
           << prediction(0) << ";" << prediction(1) << ";" << prediction(2) << ";" << prediction(3) << ";"
           << prediction(4) << ";" << prediction(5) << ";" << prediction(6) << ";" << prediction(7) << ";"
           << update(0) << ";" << update(1) << ";" << update(2) << ";" << update(3) << ";"
           << update(4) << ";" << update(5) << ";" << update(6) << ";" << update(7) << ";"
           << update(0) + update(4)  << ";" << update(1) + update(5) << ";" << update(2) + update(6) << ";" << update(3) + update(7) << "\n";
    myfile.close();

    ofstream myfile2;
    myfile2.open ("/home/catta/lane-short.txt", ios::app);
    myfile2 << msg_lines.way_id  << ";" << SensorOK << ";" << SensorBAD << ";"
           << tentative(0) << ";" << tentative(1) << ";" << tentative(2) << ";" << tentative(3) << ";"
           << update(0) + update(4)  << ";" << update(1) + update(5) << ";" << update(2) + update(6) << ";" << update(3) + update(7) << "\n";
    myfile2.close();

        std_msgs::Bool msg;
        msg.data=false;
        chatter_pub->publish(msg);

}

int main(int argc, char **argv)
{


    ros::init(argc, argv, "lane");
    ros::NodeHandle n;

    /// This sets the logger level; use this to disable all ROS prints
    if ( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
        ros::console::notifyLoggerLevelsChanged();
    else
        std::cout << "Error while setting the logger level!" << std::endl;

    chatter_pub = new ros::Publisher;
    *chatter_pub = n.advertise<std_msgs::Bool>("/sync", 1);

    howManyLanes = 4;
    //megavariabile.resize(4);
    //megavariabile.setConstant(1.0f / 4.0f);
    setStateTransitionMatrix();
    resetMegavariabile(howManyLanes);
    resetSensor(howManyLanes);

    ros::Subscriber sub = n.subscribe("/isis_line_detector/lines", 1, chatterCallback);

    ros::spin();

    return 0;
}

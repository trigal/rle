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
Eigen::Array4d megavariabile;

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

/**
 * @brief chatterCallback
 * @param msg_lines
 *
 * this should became the roadLaneCallback
 */
void chatterCallback(const road_layout_estimation::msg_lines & msg_lines)
{
    cout << "================================================================"<< endl << endl;
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    cout << megavariabile.sum() << endl;
    ROS_ASSERT( (1 - megavariabile.sum()) <= 0.00001f);

    // from excel, unified-transition matrix with 2 lanes + broken-sensor
    Eigen::MatrixXd state(4, 4);
    state << 0.693	, 0.297	, 0.007	, 0.003,
             0.297	, 0.693	, 0.003	, 0.007,
             0.07	, 0.03	, 0.63	, 0.27 ,
             0.03	, 0.07	, 0.27	, 0.63 ;
    cout << "Transition Matrix" << endl;
    cout << state.format(CleanFmt) << endl;

    //    cout << megavariabile.transpose() << " * " << state.col(0).array().transpose() << endl; //checking column/row multiplcation...

    Eigen::ArrayXd prediction(4);
    for (int i=0;i<4;i++)
        prediction[i] = (megavariabile * state.col(i).array()).sum();


    double standardLaneWidth = 3.0f; //maybe minLaneWidth
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
    std::sort (validAndSorted.begin(), validAndSorted.end(), myCompare);

    int corsie = lanesFromLines(msg_lines.number_of_lines) ; // ?goodlines? qui osm BUT FROM THE PARTICLE.
    int extra_corsie = corsie - hypothesisCurrentLanes; // quante altre penso che ci siano oltre a quella trovata OSM - TROVATE
    if (extra_corsie < 0)
        extra_corsie = 0; //non può essere minore di zero. caso in cui ho più detection di quelle che dovrebbero esserci
    double total_lenght = corsie * standardLaneWidth;
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
            if (validAndSorted.size()==1)
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
                            tentative(hypothesisCurrentLanes-corsie) = 1;

                        tot += standardLaneWidth;
                        corsie--;
                    }
                }

                /// Add noise to every guess (moved outside the if)
                //tentative += 0.001f;
                //tentative /= tentative.sum();
            }
            else
                ROS_ASSERT_MSG(1,"nope ... ");
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
    int counter=0;
    for(int i=0; i<msg_lines.lines.size();i++)
        if (msg_lines.lines.at(i).isValid)
            counter += msg_lines.lines.at(i).counter;

    double SensorOK  = 0.0f; //double(counter) / double(msg_lines.number_of_lines * 10);
    double SensorBAD = 0.0f; //1-SensorOK;
    SensorOK  = double(counter) / double(msg_lines.number_of_lines * 10) - 0.01f; //adding noise
    if (SensorOK<0.0f)
        SensorOK=0.01;
    SensorBAD = double(1.0f)-SensorOK;

    ROS_INFO_STREAM("Sensor OK/BAD:" << SensorOK << "/" << SensorBAD << " - counter: " << counter << "/" << msg_lines.number_of_lines * 10);

    Eigen::Array2d a;
    Eigen::Array2d b;
    a = (tentative * SensorOK ) ; //<< endl;
    b = (tentative * SensorBAD) ; //<< endl;
    sensor.setZero(4);
    sensor << a,b;

    cout << "sensor     :\t" <<sensor.transpose().format(CleanFmt) << endl;
    cout << "prediction :\t" <<prediction.transpose().format(CleanFmt) << endl;
    Eigen::Vector4d update; update << sensor.array() * prediction.array() ;
    //cout << "no-norm-upd:\t" << update.transpose().format(CleanFmt) << endl;
    double sum = update.sum();
    //cout << "norm. fact :\t" << sum << endl;
    update /= sum;
    cout << "update     :\t" <<update.transpose().format(CleanFmt) << endl;
    cout << "summarize  :\t" <<update(0)+update(2) << " | " << update(1)+update(3) << endl;

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

    megavariabile.resize(4);
    megavariabile.setConstant(1.0f / 4.0f);

    ros::Subscriber sub = n.subscribe("/kitti_player/lanes", 1, chatterCallback);

    ros::spin();

    return 0;
}

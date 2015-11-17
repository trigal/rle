#include "ros/ros.h"
#include "std_msgs/String.h"
#include "road_layout_estimation/msg_lines.h"
#include "road_layout_estimation/msg_lineInfo.h"
#include <limits>
#include <algorithm>    // std::sort
#include <vector>       // std::vector

#include <eigen3/Eigen/Core>

using namespace std;

double numb = 4;
Eigen::Array4f corsia(1 / numb, 1 / numb, 1 / numb, 1 / numb);
//Eigen::Array4f sensor(0.25,0.25,0.25,0.25);
Eigen::Array4f sensor(0.9, 0.1, 0.0, 0.0);

bool myCompare(road_layout_estimation::msg_lineInfo a, road_layout_estimation::msg_lineInfo b)
{
    if (std::fabs(a.offset) < std::fabs(b.offset))
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

void chatterCallback(const road_layout_estimation::msg_lines & msg_lines)
{
    std::cout << corsia.transpose() << endl;

    Eigen::MatrixXf state(4, 4);
    state << 0.91 , 0.09 , 0    , 0   ,
          0.09 , 0.82 , 0.09 , 0   ,
          0    , 0.09 , 0.82 , 0.09,
          0    , 0    , 0.1  , 0.9 ;

    //Eigen::MatrixXf test(5,5);
    //test << 1,1,1,1,1,
    //        2,2,2,2,2,
    //        3,3,3,3,3,
    //        4,4,4,4,4,
    //        5,5,5,5,5;
    //
    //Eigen::ArrayXf vect(5);
    //vect << 1,2,3,4,5;
    //
    //std::cout << std::endl;
    //Eigen::MatrixXf ris = vect * test.row(1).transpose().array();
    //
    //std::cout << ris.transpose() << std::endl ;

    Eigen::Array4f temp;
    temp[0] = (corsia * state.row(0).transpose().array()).sum();
    temp[1] = (corsia * state.row(1).transpose().array()).sum();
    temp[2] = (corsia * state.row(2).transpose().array()).sum();
    temp[3] = (corsia * state.row(3).transpose().array()).sum();

    std::cout << temp.transpose() << endl ;

    double standardLaneWidth = 4.0f;
    unsigned int hypothesisCurrentLanesNaive = 0.0f; //hypothesis of current lane
    unsigned int hypothesisCurrentLanes = 0.0f; //hypothesis of current lane

    /// Heuristic for number of lanestd::fmod(msg_lines.naive_width, standardLaneWidth)s given the detected/naive width
//    if (std::fmod(msg_lines.naive_width, standardLaneWidth) >= (standardLaneWidth / 1.0f))
//        hypothesisCurrentLanesNaive = round(msg_lines.naive_width / standardLaneWidth );
//    else
//        hypothesisCurrentLanesNaive = round(msg_lines.naive_width / standardLaneWidth ) + 1;
//
//    if (std::fmod(msg_lines.naive_width, standardLaneWidth)  >= (standardLaneWidth / 1.0f))
//        hypothesisCurrentLanes = round(msg_lines.width / standardLaneWidth );
//    else
//        hypothesisCurrentLanes = round(msg_lines.width / standardLaneWidth ) + 1;

    hypothesisCurrentLanesNaive = round(msg_lines.naive_width / standardLaneWidth );
    hypothesisCurrentLanes = round(msg_lines.width / standardLaneWidth );

    //ROS_INFO_STREAM(hypothesisCurrentLanesNaive << "\t" << msg_lines.naive_width << "\t\t" << hypothesisCurrentLanes << "\t" << msg_lines.width);

    vector <road_layout_estimation::msg_lineInfo> tmp = msg_lines.lines;
    vector <road_layout_estimation::msg_lineInfo>::iterator rm;
    rm = tmp.erase(std::remove_if(tmp.begin(), tmp.end(), isFalse));
    std::sort (tmp.begin(), rm, myCompare);

    if (msg_lines.width > standardLaneWidth)
    {
        // se ci sono almeno due linee, quindi almeno una corsia
        if (msg_lines.goodLines > 1)
        {

            int corsie = lanesFromLines(msg_lines.number_of_lines); // ?goodlines? qui osm
            int extra_corsie = corsie - hypothesisCurrentLanes; // quante altre penso che ci siano
            double total_lenght = corsie * standardLaneWidth;

            Eigen::ArrayXi tentative(corsie);
            tentative.setZero(corsie);

            while (extra_corsie+1)
            {
                /// In quale delle corsie su hypothesisCurrentLanes sono?
                int toadd=corsie;
                for (int i = 0; i < msg_lines.goodLines; i++)
                {
                    if (std::fabs(tmp[i].offset) < standardLaneWidth)
                    {
                        tentative(toadd-1)=tentative(toadd-1)+1;
                        break;
                    }
                    else
                        toadd--;
                }

                corsie--;
                extra_corsie--;
            }

            ROS_ERROR_STREAM(tentative[0] << " " << tentative[1] << " " << tentative[2]);


        }
        else
        {
            sensor << 0.25, 0.25, 0.25, 0.25 ;
        }
    }

    //sensor << 0.9, 0.1, 0, 0;
    corsia = temp * sensor;
    corsia /= corsia.sum();

    std::cout << corsia << endl << endl;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/kitti_player/lanes", 1, chatterCallback);

    ros::spin();

    return 0;
}

#include <algorithm>
#include <boost/foreach.hpp>
#include <csv.h>
#include <eigen3/Eigen/Core>
#include <fstream>
#include <iomanip> // setprecision
#include <limits>
#include <random>
#include "road_layout_estimation/msg_lineInfo.h"
#include "road_layout_estimation/msg_lines.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/console.h>
#include "ros/ros.h"
#include <sstream> // stringstream
#include <stdio.h>
#include <std_msgs/Bool.h>
#include <string>
#include <vector>
#include <boost/math/distributions/normal.hpp>

#define foreach BOOST_FOREACH
// #define VERBOSE_MODEL
using namespace std;
using boost::math::normal;

string multiprocess;

static Eigen::VectorXd sensor;
static Eigen::ArrayXd megavariabile;
static Eigen::MatrixXd stateTransitionMatrix;

static int howManyLanes;                   ///< The number of lanes in the current hypothesis
static double standardLaneWidth = 3.5f;    // maybe minLaneWidth
static int MAX_COUNT = 10;
static int plus_corsie_continue = 2;
static string testname;
static double average_weight = 0.6;        //media tra stato precedente e quello che dice il sensore, nel caso di bad state

static ros::Publisher *sync_publisher ;

bool isFalse(road_layout_estimation::msg_lineInfo a);
bool myCompare(road_layout_estimation::msg_lineInfo a, road_layout_estimation::msg_lineInfo b);
double evaluate(bool deletefiles_ = true);
inline double normalDistributionAt(double x, double mean, double sigma);
int lanesFromLines(int goodLines);
void deletefiles();
void executeTest(const road_layout_estimation::msg_lines & msg_lines);
void makeTransitionMatrix(double sigma, double sigma2, double P1, double P2);
void makeTransitionMatrixS2(double sigma, double P1, double P2);
void resetMegavariabile(int lanes);
void resetSensor(int lanes);
void setStateTransitionMatrix();
void setTestName(int lanes, double sigma1, double P1, double P2, double sigma2 = -1.0f, double weight = 0.6f);
void setupEnv(double sigma1, double sigma2, double P1, double P2, int pluscorsie, int lanes_number, double av_weight);
void oneShot(rosbag::View &view);


ROS_DEPRECATED void makeTransitionMatrix(double sigma, double P1, double P2)
{
    // just for printing purposes
    Eigen::IOFormat CleanFmt(Eigen::FullPrecision, 0, ", ", "\n", "[", "]");

    // resize the final transition matrix
    stateTransitionMatrix.resize(8, 8);

    // create support matrices. populate a vector with the PDF normal values
    Eigen::MatrixXd buildingTransition, a1, b1, a2, b2;
    vector<double> values;
    buildingTransition.resize(4, 4);
    for (int mean = 1; mean < 5; mean++)
        for (int x = 1; x < 5; x++)
            values.push_back(normalDistributionAt(x, mean, sigma));

    // push the values from the vector into the support matrix (eigen)
    buildingTransition << values[0], values[1], values[2], values[3],
                       values[4], values[5], values[6], values[7],
                       values[8], values[9], values[10], values[11],
                       values[12], values[13], values[14], values[15];

    // print it (debug)
#ifdef VERBOSE_MODEL
    cout << buildingTransition.format(CleanFmt) << endl << endl;
#endif

    // normalize each row using the sum. WARNING: here are the cols ...
    for (int i = 0; i < 4; i++)
        buildingTransition.col(i) /= buildingTransition.col(i).sum();

    // print it (debug)
#ifdef VERBOSE_MODEL
    cout << buildingTransition.format(CleanFmt) << endl << endl;
#endif

    // The final matrix will be like this. First create the 4 sub-matrices
    // [    A1    |    B1    ]
    // [    A2    |    B2    ]

    // in the support matrix a1 and b1, multiply the normalized matrix by P1
    a1 = buildingTransition * P1;
    b1 = buildingTransition * (1.0f - P1);

    // print it (debug)
    //cout << a1.format(CleanFmt) << endl << endl;
    //cout << b1.format(CleanFmt) << endl << endl;

    a2 = buildingTransition * P2;
    b2 = buildingTransition * (1.0f - P2);

    // create the [ A1 | B1 ] and [ A2 | B2 ] as c1 and c2
    Eigen::MatrixXd c1(a1.rows() + b1.rows(), +a1.cols());
    Eigen::MatrixXd c2(a2.rows() + b2.rows(), +a2.cols());
    c1 << a1, b1;
    c2 << a2, b2;

#ifdef VERBOSE_MODEL
    // print it (debug)
    cout << c1.transpose().format(CleanFmt) << endl << endl;
    cout << c2.transpose().format(CleanFmt) << endl << endl;
#endif

    // concatenate c1 and c2 into stateTransitionMatrix
    stateTransitionMatrix << c1.transpose(), c2.transpose();

#ifdef VERBOSE_MODEL
    // print it (debug)
    cout << stateTransitionMatrix.format(CleanFmt) << endl << endl;
#endif

}
void makeTransitionMatrixS2(double sigma1, double sigma2, double P1, double P2)
{
    // just for printing purposes
    Eigen::IOFormat CleanFmt(Eigen::FullPrecision, 0, ", ", "\n", "[", "]");

    // resize the final transition matrix
    stateTransitionMatrix.resize(8, 8);

    // create support matrices. populate a vector with the PDF normal values
    Eigen::MatrixXd buildingTransition1, buildingTransition2, a1, b1, a2, b2;
    vector<double> valuesForTransition1, valuesForTransition2;

    buildingTransition1.resize(4, 4);
    buildingTransition2.resize(4, 4);

    for (int mean = 1; mean < 5; mean++)
        for (int x = 1; x < 5; x++)
        {
            valuesForTransition1.push_back(normalDistributionAt(x, mean, sigma1));
            valuesForTransition2.push_back(normalDistributionAt(x, mean, sigma2));
        }

    // push the values from the vector into the support matrix (eigen)
    buildingTransition1 << valuesForTransition1[0], valuesForTransition1[1], valuesForTransition1[2], valuesForTransition1[3],
                        valuesForTransition1[4], valuesForTransition1[5], valuesForTransition1[6], valuesForTransition1[7],
                        valuesForTransition1[8], valuesForTransition1[9], valuesForTransition1[10], valuesForTransition1[11],
                        valuesForTransition1[12], valuesForTransition1[13], valuesForTransition1[14], valuesForTransition1[15];

    buildingTransition2 << valuesForTransition2[0],  valuesForTransition2 [1],  valuesForTransition2[2],  valuesForTransition2[3],
                        valuesForTransition2[4],  valuesForTransition2 [5],  valuesForTransition2[6],  valuesForTransition2[7],
                        valuesForTransition2[8],  valuesForTransition2 [9],  valuesForTransition2[10], valuesForTransition2[11],
                        valuesForTransition2[12], valuesForTransition2 [13], valuesForTransition2[14], valuesForTransition2[15];
#ifdef VERBOSE_MODEL
    // print it (debug)
    cout << buildingTransition1.format(CleanFmt) << endl << endl;
    cout << buildingTransition2.format(CleanFmt) << endl << endl;
#endif

    // normalize each row using the sum. WARNING: here are the cols ...
    for (int i = 0; i < 4; i++)
    {
        buildingTransition1.col(i) /= buildingTransition1.col(i).sum();
        buildingTransition2.col(i) /= buildingTransition2.col(i).sum();
    }

#ifdef VERBOSE_MODEL
    // print it (debug)
    cout << buildingTransition1.format(CleanFmt) << endl << endl;
#endif

    // The final matrix will be like this. First create the 4 sub-matrices
    // [    A1    |    B1    ]
    // [    A2    |    B2    ]

    // in the support matrix a1 and b1, multiply the normalized matrix by P1
    a1 = buildingTransition1 * P1;
    b1 = buildingTransition1 * (1.0f - P1);

    // print it (debug)
    //cout << a1.format(CleanFmt) << endl << endl;
    //cout << b1.format(CleanFmt) << endl << endl;

    a2 = buildingTransition2 * P2;
    b2 = buildingTransition2 * (1.0f - P2);

    // create the [ A1 | B1 ] and [ A2 | B2 ] as c1 and c2
    Eigen::MatrixXd c1(a1.rows() + b1.rows(), +a1.cols());
    Eigen::MatrixXd c2(a2.rows() + b2.rows(), +a2.cols());
    c1 << a1, b1;
    c2 << a2, b2;

#ifdef VERBOSE_MODEL
    // print it (debug)
    cout << c1.transpose().format(CleanFmt) << endl << endl;
    cout << c2.transpose().format(CleanFmt) << endl << endl;
#endif

    // concatenate c1 and c2 into stateTransitionMatrix
    stateTransitionMatrix << c1.transpose(), c2.transpose();

#ifdef VERBOSE_MODEL
    // print it (debug)
    cout << stateTransitionMatrix.format(CleanFmt) << endl << endl;
#endif

}
inline double normalDistributionAt(double x, double mean, double sigma)
{
    normal distribution(mean, sigma);
    return pdf (distribution, x);
}

/// LANE ALGORITHM PART
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

        ///0.9 0.2 SENSORBAD (YELLOW BOX)
        if (EXEC_TEST == 1)
        {
            ROS_INFO_STREAM("Executing test " + std::to_string(EXEC_TEST));
            testname = "0.72+0902";
            stateTransitionMatrix << 0.64173370, 0.24461052, 0.01354678, 0.00010900, 0.07130374, 0.02717895, 0.00150520, 0.00001211,
                                  0.19235402, 0.50463920, 0.19235402, 0.01065276, 0.02137267, 0.05607102, 0.02137267, 0.00118364,
                                  0.01065276, 0.19235402, 0.50463920, 0.19235402, 0.00118364, 0.02137267, 0.05607102, 0.02137267,
                                  0.00010900, 0.01354678, 0.24461052, 0.64173370, 0.00001211, 0.00150520, 0.02717895, 0.07130374,
                                  0.14260749, 0.05435789, 0.00301040, 0.00002422, 0.57042995, 0.21743158, 0.01204158, 0.00009689,
                                  0.04274534, 0.11214204, 0.04274534, 0.00236728, 0.17098135, 0.44856817, 0.17098135, 0.00946912,
                                  0.00236728, 0.04274534, 0.11214204, 0.04274534, 0.00946912, 0.17098135, 0.44856817, 0.17098135,
                                  0.00002422, 0.00301040, 0.05435789, 0.14260749, 0.00009689, 0.01204158, 0.21743158, 0.57042995;
        }

        if (EXEC_TEST == 2)
        {
            ROS_INFO_STREAM("Executing test " + std::to_string(EXEC_TEST));
            testname = "0.6+0902";
            stateTransitionMatrix << 0.71814898, 0.17907203, 0.00277631, 0.00000268, 0.07979433, 0.01989689, 0.00030848, 0.00000030,
                                  0.14935540, 0.59897362, 0.14935540, 0.00231558, 0.01659504, 0.06655262, 0.01659504, 0.00025729,
                                  0.00231558, 0.14935540, 0.59897362, 0.14935540, 0.00025729, 0.01659504, 0.06655262, 0.01659504,
                                  0.00000268, 0.00277631, 0.17907203, 0.71814898, 0.00000030, 0.00030848, 0.01989689, 0.07979433,
                                  0.15958866, 0.03979379, 0.00061696, 0.00000059, 0.63835465, 0.15917514, 0.00246783, 0.00000238,
                                  0.03319009, 0.13310525, 0.03319009, 0.00051457, 0.13276035, 0.53242100, 0.13276035, 0.00205830,
                                  0.00051457, 0.03319009, 0.13310525, 0.03319009, 0.00205830, 0.13276035, 0.53242100, 0.13276035,
                                  0.00000059, 0.00061696, 0.03979379, 0.15958866, 0.00000238, 0.00246783, 0.15917514, 0.63835465;
        }
        //
        if (EXEC_TEST == 3)
        {
            ROS_INFO_STREAM("Executing test " + std::to_string(EXEC_TEST));
            testname = "0.8+0902";
            stateTransitionMatrix << 0.59894023, 0.27421482, 0.02631560, 0.00052936, 0.06654891, 0.03046831, 0.00292396, 0.00005882,
                                  0.21027213, 0.45927655, 0.21027213, 0.02017920, 0.02336357, 0.05103073, 0.02336357, 0.00224213,
                                  0.02017920, 0.21027213, 0.45927655, 0.21027213, 0.00224213, 0.02336357, 0.05103073, 0.02336357,
                                  0.00052936, 0.02631560, 0.27421482, 0.59894023, 0.00005882, 0.00292396, 0.03046831, 0.06654891,
                                  0.13309783, 0.06093663, 0.00584791, 0.00011764, 0.53239131, 0.24374650, 0.02339164, 0.00047054,
                                  0.04672714, 0.10206145, 0.04672714, 0.00448427, 0.18690856, 0.40824582, 0.18690856, 0.01793707,
                                  0.00448427, 0.04672714, 0.10206145, 0.04672714, 0.01793707, 0.18690856, 0.40824582, 0.18690856,
                                  0.00011764, 0.00584791, 0.06093663, 0.13309783, 0.00047054, 0.02339164, 0.24374650, 0.53239131;
        }
        //
        if (EXEC_TEST == 4)
        {
            ROS_INFO_STREAM("Executing test " + std::to_string(EXEC_TEST));
            testname = "0.5+0902";
            stateTransitionMatrix << 0.79248320, 0.10725094, 0.00026585, 0.00000001, 0.08805369, 0.01191677, 0.00002954, 0.00000000,
                                  0.09583098, 0.70810050, 0.09583098, 0.00023754, 0.01064789, 0.07867783, 0.01064789, 0.00002639,
                                  0.00023754, 0.09583098, 0.70810050, 0.09583098, 0.00002639, 0.01064789, 0.07867783, 0.01064789,
                                  0.00000001, 0.00026585, 0.10725094, 0.79248320, 0.00000000, 0.00002954, 0.01191677, 0.08805369,
                                  0.17610738, 0.02383354, 0.00005908, 0.00000000, 0.70442951, 0.09533417, 0.00023631, 0.00000001,
                                  0.02129577, 0.15735567, 0.02129577, 0.00005279, 0.08518309, 0.62942266, 0.08518309, 0.00021115,
                                  0.00005279, 0.02129577, 0.15735567, 0.02129577, 0.00021115, 0.08518309, 0.62942266, 0.08518309,
                                  0.00000000, 0.00005908, 0.02383354, 0.17610738, 0.00000001, 0.00023631, 0.09533417, 0.70442951;
        }
        //
        if (EXEC_TEST == 5)
        {
            ROS_INFO_STREAM("Executing test " + std::to_string(EXEC_TEST));
            testname = "0.9+0902";
            stateTransitionMatrix << 0.55284884, 0.29821082, 0.04680307, 0.00213727, 0.06142765, 0.03313454, 0.00520034, 0.00023747,
                                  0.22439233, 0.41599780, 0.22439233, 0.03521754, 0.02493248, 0.04622198, 0.02493248, 0.00391306,
                                  0.03521754, 0.22439233, 0.41599780, 0.22439233, 0.00391306, 0.02493248, 0.04622198, 0.02493248,
                                  0.00213727, 0.04680307, 0.29821082, 0.55284884, 0.00023747, 0.00520034, 0.03313454, 0.06142765,
                                  0.12285530, 0.06626907, 0.01040068, 0.00047495, 0.49142119, 0.26507628, 0.04160273, 0.00189980,
                                  0.04986496, 0.09244395, 0.04986496, 0.00782612, 0.19945985, 0.36977582, 0.19945985, 0.03130448,
                                  0.00782612, 0.04986496, 0.09244395, 0.04986496, 0.03130448, 0.19945985, 0.36977582, 0.19945985,
                                  0.00047495, 0.01040068, 0.06626907, 0.12285530, 0.00189980, 0.04160273, 0.26507628, 0.49142119;
        }
        //
        if (EXEC_TEST == 6)
        {
            ROS_INFO_STREAM("Executing test " + std::to_string(EXEC_TEST));
            testname = "1.5+0902";
            stateTransitionMatrix << 0.38343804, 0.30703318, 0.15763609, 0.05189270, 0.04260423, 0.03411480, 0.01751512, 0.00576586,
                                  0.23921754, 0.29874655, 0.23921754, 0.12281838, 0.02657973, 0.03319406, 0.02657973, 0.01364649,
                                  0.12281838, 0.23921754, 0.29874655, 0.23921754, 0.01364649, 0.02657973, 0.03319406, 0.02657973,
                                  0.05189270, 0.15763609, 0.30703318, 0.38343804, 0.00576586, 0.01751512, 0.03411480, 0.04260423,
                                  0.08520845, 0.06822960, 0.03503024, 0.01153171, 0.34083381, 0.27291838, 0.14012097, 0.04612684,
                                  0.05315945, 0.06638812, 0.05315945, 0.02729297, 0.21263781, 0.26555249, 0.21263781, 0.10917189,
                                  0.02729297, 0.05315945, 0.06638812, 0.05315945, 0.10917189, 0.21263781, 0.26555249, 0.21263781,
                                  0.01153171, 0.03503024, 0.06822960, 0.08520845, 0.04612684, 0.14012097, 0.27291838, 0.34083381;
        }
        //
        if (EXEC_TEST == 7)
        {
            ROS_INFO_STREAM("Executing test " + std::to_string(EXEC_TEST));
            testname = "2.0+0902";
            stateTransitionMatrix << 0.31986580, 0.28228057, 0.19400841, 0.10384522, 0.03554064, 0.03136451, 0.02155649, 0.01153836,
                                  0.23557510, 0.26694156, 0.23557510, 0.16190824, 0.02617501, 0.02966017, 0.02617501, 0.01798980,
                                  0.16190824, 0.23557510, 0.26694156, 0.23557510, 0.01798980, 0.02617501, 0.02966017, 0.02617501,
                                  0.10384522, 0.19400841, 0.28228057, 0.31986580, 0.01153836, 0.02155649, 0.03136451, 0.03554064,
                                  0.07108129, 0.06272902, 0.04311298, 0.02307672, 0.28432515, 0.25091607, 0.17245192, 0.09230686,
                                  0.05235002, 0.05932035, 0.05235002, 0.03597961, 0.20940009, 0.23728139, 0.20940009, 0.14391844,
                                  0.03597961, 0.05235002, 0.05932035, 0.05235002, 0.14391844, 0.20940009, 0.23728139, 0.20940009,
                                  0.02307672, 0.04311298, 0.06272902, 0.07108129, 0.09230686, 0.17245192, 0.25091607, 0.28432515;
        }
        //
        if (EXEC_TEST == 8)
        {
            ROS_INFO_STREAM("Executing test " + std::to_string(EXEC_TEST));
            testname = "3.0+0902";
            stateTransitionMatrix << 0.26839813, 0.25389375, 0.21491642, 0.16279170, 0.02982201, 0.02821042, 0.02387960, 0.01808797,
                                  0.23055585, 0.24372699, 0.23055585, 0.19516131, 0.02561732, 0.02708078, 0.02561732, 0.02168459,
                                  0.19516131, 0.23055585, 0.24372699, 0.23055585, 0.02168459, 0.02561732, 0.02708078, 0.02561732,
                                  0.16279170, 0.21491642, 0.25389375, 0.26839813, 0.01808797, 0.02387960, 0.02821042, 0.02982201,
                                  0.05964403, 0.05642083, 0.04775920, 0.03617593, 0.23857612, 0.22568334, 0.19103682, 0.14470373,
                                  0.05123463, 0.05416155, 0.05123463, 0.04336918, 0.20493853, 0.21664621, 0.20493853, 0.17347672,
                                  0.04336918, 0.05123463, 0.05416155, 0.05123463, 0.17347672, 0.20493853, 0.21664621, 0.20493853,
                                  0.03617593, 0.04775920, 0.05642083, 0.05964403, 0.14470373, 0.19103682, 0.22568334, 0.23857612;
        }
        //
        //
        //
        /////=======================
        /////0.9 0.1 SENSORBAD (YELLOW BOX)
        if (EXEC_TEST == 9)
        {
            ROS_INFO_STREAM("Executing test " + std::to_string(EXEC_TEST));
            testname = "0.9+0901";
            stateTransitionMatrix << 0.55284884, 0.29821082, 0.04680307, 0.00213727, 0.06142765, 0.03313454, 0.00520034, 0.00023747,
                                  0.22439233, 0.41599780, 0.22439233, 0.03521754, 0.02493248, 0.04622198, 0.02493248, 0.00391306,
                                  0.03521754, 0.22439233, 0.41599780, 0.22439233, 0.00391306, 0.02493248, 0.04622198, 0.02493248,
                                  0.00213727, 0.04680307, 0.29821082, 0.55284884, 0.00023747, 0.00520034, 0.03313454, 0.06142765,
                                  0.06142765, 0.03313454, 0.00520034, 0.00023747, 0.55284884, 0.29821082, 0.04680307, 0.00213727,
                                  0.02493248, 0.04622198, 0.02493248, 0.00391306, 0.22439233, 0.41599780, 0.22439233, 0.03521754,
                                  0.00391306, 0.02493248, 0.04622198, 0.02493248, 0.03521754, 0.22439233, 0.41599780, 0.22439233,
                                  0.00023747, 0.00520034, 0.03313454, 0.06142765, 0.00213727, 0.04680307, 0.29821082, 0.55284884;
        }
        //
        //
        //
        /////==============
        /////0.8 0.2 SENSORBAD (YELLOW BOX)
        if (EXEC_TEST == 10)
        {
            ROS_INFO_STREAM("Executing test " + std::to_string(EXEC_TEST));
            testname = "0.9+0802";
            stateTransitionMatrix << 0.49142119, 0.26507628, 0.04160273, 0.00189980, 0.12285530, 0.06626907, 0.01040068, 0.00047495,
                                  0.19945985, 0.36977582, 0.19945985, 0.03130448, 0.04986496, 0.09244395, 0.04986496, 0.00782612,
                                  0.03130448, 0.19945985, 0.36977582, 0.19945985, 0.00782612, 0.04986496, 0.09244395, 0.04986496,
                                  0.00189980, 0.04160273, 0.26507628, 0.49142119, 0.00047495, 0.01040068, 0.06626907, 0.12285530,
                                  0.12285530, 0.06626907, 0.01040068, 0.00047495, 0.49142119, 0.26507628, 0.04160273, 0.00189980,
                                  0.04986496, 0.09244395, 0.04986496, 0.00782612, 0.19945985, 0.36977582, 0.19945985, 0.03130448,
                                  0.00782612, 0.04986496, 0.09244395, 0.04986496, 0.03130448, 0.19945985, 0.36977582, 0.19945985,
                                  0.00047495, 0.01040068, 0.06626907, 0.12285530, 0.00189980, 0.04160273, 0.26507628, 0.49142119;
        }
        //
        /////==================,
        /////0.8 + 0.1 SENSORBAD
        if (EXEC_TEST == 11)
        {
            ROS_INFO_STREAM("Executing test " + std::to_string(EXEC_TEST));
            testname = "0.9+0801";
            stateTransitionMatrix << 0.49142119, 0.26507628, 0.04160273, 0.00189980, 0.12285530, 0.06626907, 0.01040068, 0.00047495,
                                  0.19945985, 0.36977582, 0.19945985, 0.03130448, 0.04986496, 0.09244395, 0.04986496, 0.00782612,
                                  0.03130448, 0.19945985, 0.36977582, 0.19945985, 0.00782612, 0.04986496, 0.09244395, 0.04986496,
                                  0.00189980, 0.04160273, 0.26507628, 0.49142119, 0.00047495, 0.01040068, 0.06626907, 0.12285530,
                                  0.06142765, 0.03313454, 0.00520034, 0.00023747, 0.55284884, 0.29821082, 0.04680307, 0.00213727,
                                  0.02493248, 0.04622198, 0.02493248, 0.00391306, 0.22439233, 0.41599780, 0.22439233, 0.03521754,
                                  0.00391306, 0.02493248, 0.04622198, 0.02493248, 0.03521754, 0.22439233, 0.41599780, 0.22439233,
                                  0.00023747, 0.00520034, 0.03313454, 0.06142765, 0.00213727, 0.04680307, 0.29821082, 0.55284884;
        }
        //
        /////============
        /////0.6 + 0.4 SENSORBAD
        if (EXEC_TEST == 12)
        {
            ROS_INFO_STREAM("Executing test " + std::to_string(EXEC_TEST));
            testname = "0.9+0604";
            stateTransitionMatrix << 0.36856590, 0.19880721, 0.03120205, 0.00142485, 0.24571060, 0.13253814, 0.02080136, 0.00094990,
                                  0.14959489, 0.27733186, 0.14959489, 0.02347836, 0.09972993, 0.18488791, 0.09972993, 0.01565224,
                                  0.02347836, 0.14959489, 0.27733186, 0.14959489, 0.01565224, 0.09972993, 0.18488791, 0.09972993,
                                  0.00142485, 0.03120205, 0.19880721, 0.36856590, 0.00094990, 0.02080136, 0.13253814, 0.24571060,
                                  0.24571060, 0.13253814, 0.02080136, 0.00094990, 0.36856590, 0.19880721, 0.03120205, 0.00142485,
                                  0.09972993, 0.18488791, 0.09972993, 0.01565224, 0.14959489, 0.27733186, 0.14959489, 0.02347836,
                                  0.01565224, 0.09972993, 0.18488791, 0.09972993, 0.02347836, 0.14959489, 0.27733186, 0.14959489,
                                  0.00094990, 0.02080136, 0.13253814, 0.24571060, 0.00142485, 0.03120205, 0.19880721, 0.36856590;
        }





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
void executeTest(const road_layout_estimation::msg_lines & msg_lines)
{
#ifdef VERBOSE_MODEL
    cout << "================================================================" << endl << endl;
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    cout << msg_lines.header.seq << endl;
#endif

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
                    tentative(corsia_attuale) += plus_corsie_continue;
            }

            /// Add noise to every guess (moved outside the if)
            //tentative += 0.001f;
            //tentative /= tentative.sum();
        }
        else if (validAndSorted.at(i).continuous == true)
        {
            if (validAndSorted.at(i).offset < 0)
            {
                tentative(0) += plus_corsie_continue;
            }
            else
                tentative(howManyLanes - 1) += plus_corsie_continue;
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
#ifdef VERBOSE_MODEL
        ROS_WARN_STREAM(s);
#endif
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

#ifdef VERBOSEMODEL
    ROS_INFO_STREAM("Sensor OK/BAD:" << SensorOK << "/" << SensorBAD << " - counter: " << counter << "/" << (howManyLanes + 1) * MAX_COUNT);
#endif

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
    //TODO:  PARAMETRIZZARE ANCHE QUESTO!!
    double weight = average_weight; //0.6; //                      Wright è la media tra lo stato precedente e quello che dice il sensore, nel caso di bad state
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

#ifdef VERBOSE_MODEL
    //cout << "SensorState:\t" << sensor_bad_state << endl;
    cout << "Tentative  :\t" << tentative.transpose().format(CleanFmt) << endl;
    cout << "sensor     :\t" << sensor.transpose().format(CleanFmt) << endl;
    cout << "prediction :\t" << prediction.transpose().format(CleanFmt) << endl;
#endif

    Eigen::VectorXd update;
    update.setZero(howManyLanes * 2);
    update << sensor.array() * prediction.array() ;
    //cout << "no-norm-upd:\t" << update.transpose().format(CleanFmt) << endl;
    double sum = update.sum();
    //cout << "norm. fact :\t" << sum << endl;
    update /= sum;
#ifdef VERBOSE_MODEL
    cout << "update     :\t" << update.transpose().format(CleanFmt) << endl;
    if (howManyLanes == 2)
        cout << "summarize  :\t" << update(0) + update(2) << " | " << update(1) + update(3) << endl;
    else if (howManyLanes == 3)
        cout << "summarize  :\t" << update(0) + update(3) << " | " << update(1) + update(4) << " | " << update(2) + update(5) << endl;
    else if (howManyLanes == 4)
        cout << "summarize  :\t" << update(0) + update(4) << " | " << update(1) + update(5) << " | " << update(2) + update(6) << " | " << update(3) + update(7) << endl;
#endif

    ROS_ASSERT( (1 - update.sum()) <= 0.00001f);

    megavariabile = update;

    ///STATS
    ofstream myfile;
    ROS_DEBUG_STREAM("Saving results in: " << SAVEPATH + testname + ".txt");
    myfile.open (SAVEPATH + testname + ".txt", ios::app);
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
    ROS_DEBUG_STREAM("Saving results in: " << SAVEPATH + testname + ".short.txt");
    myfile2.open (SAVEPATH + testname + ".short.txt", ios::app);
    myfile2 << msg_lines.way_id  << ";" << SensorOK << ";" << SensorBAD << ";"
            << tentative(0) << ";" << tentative(1) << ";" << tentative(2) << ";" << tentative(3) << ";"
            << update(0) + update(4)  << ";" << update(1) + update(5) << ";" << update(2) + update(6) << ";" << update(3) + update(7) << "\n";
    myfile2.close();

#if SYNCMODE
    std_msgs::Bool msg;
    msg.data = false;
    sync_publisher->publish(msg);
#endif

}
/// END

/// EVALUATION PART (returns fitnessgain)
double evaluate(bool deletefiles_)
{
    string testfile_ = SAVEPATH + testname + ".short.txt";
    string gtfile_ = SAVEPATH GTFILE;

    // Check lines one each file.
    std::ifstream check_testfile_(testfile_);
    std::ifstream check_getfile_(gtfile_);
    std::string line;
    int lines_testfile_ = 0;
    int lines_gtfile_ = 0;
    while (std::getline(check_testfile_, line))
        ++lines_testfile_;
    while (std::getline(check_getfile_, line))
        ++lines_gtfile_;
    ROS_INFO_STREAM("Lines inside testfile: " << lines_testfile_);
    ROS_INFO_STREAM("Lines inside gtfile  : " << lines_gtfile_);
    if (lines_testfile_ > lines_gtfile_)
        ROS_WARN_STREAM("Lines inside " + testname + " are >>> than " GTFILE);
    if (lines_testfile_ < lines_gtfile_)
        ROS_WARN_STREAM("Lines inside " GTFILE  " are >>> than " + testname);
    if (lines_testfile_ == lines_gtfile_)
        ROS_INFO_STREAM("Good! Same number of experiments and GT!");

    io::CSVReader<11, io::trim_chars<>, io::no_quote_escape<';'> > testfile(testfile_);
    io::CSVReader<3, io::trim_chars<>, io::no_quote_escape<';'> > gtfile(gtfile_);

    testfile.set_header("seq", "v1", "v2", "tentative1", "tentative2", "tentative3", "tentative4", "summarize1", "summarize2", "summarize3", "summarize4");
    gtfile.set_header("seq", "GT", "GTFLAG");

    int testfile_seq, gtfile_seq;
    int GT, GTFLAG;
    double v1, v2;
    vector<double> tentative(4);
    vector<double> summarize(4);

    int total = 0;
    int detector_total = 0;
    int model_total = 0;

    while (testfile.read_row(testfile_seq, v1, v2, tentative[0], tentative[1], tentative[2], tentative[3], summarize[0], summarize[1], summarize[2], summarize[3]))
    {
        if (!gtfile.read_row(gtfile_seq, GT, GTFLAG))
        {
            deletefiles();
            ROS_ERROR_STREAM("Wrong GT File");
            ROS_ASSERT(0);
            return -1;
        }

        auto tentative_biggest = std::max_element(std::begin(tentative), std::end(tentative));
        auto tentative_unique = std::count (std::begin(tentative), std::end(tentative), *tentative_biggest);
        auto tentative_position = std::distance(std::begin(tentative), tentative_biggest) + 1;
        ROS_DEBUG_STREAM ("Max element is " << *tentative_biggest << " at position " << std::distance(std::begin(tentative), tentative_biggest) << " and appears " << std::count (std::begin(tentative), std::end(tentative), *tentative_biggest) << std::endl);

        auto summarize_biggest = std::max_element(std::begin(summarize), std::end(summarize));
        auto summarize_unique = std::count (std::begin(summarize), std::end(summarize), *summarize_biggest);
        auto summarize_position = std::distance(std::begin(summarize), summarize_biggest) + 1;


        // if unique, check Ground Truth
        if ( (tentative_unique == 1) && (tentative_position == GT) && (GTFLAG == false) )
            detector_total++;

        if ( (summarize_unique == 1) && (summarize_position == GT) && (GTFLAG == false) )
            model_total++;

        total++;
    }

    double detector_accuracy = static_cast<double>(detector_total) / static_cast<double>(total);
    double model_accuracy = static_cast<double>(model_total   ) / static_cast<double>(total);
    double fitness_gain = static_cast<double>(model_total - detector_total) / static_cast<double>(detector_total);

    ROS_INFO_STREAM("Detector Total:\t" << detector_total << "/" << total << ", accuracy " << detector_accuracy);
    ROS_INFO_STREAM("Model Total:\t"    << model_total    << "/" << total << ", accuracy " << model_accuracy);
    ROS_INFO_STREAM("Model Gain: \t"    << fitness_gain);

    if (deletefiles_)
        deletefiles();
//    return fitness_gain;
    return model_accuracy;

}

/// END

/// MISC FUNCTIONS
void deletefiles()
{
    string file_1 = SAVEPATH + testname + ".txt";
    string file_2 = SAVEPATH + testname + ".short.txt";
    if ( remove(file_1.c_str()) != 0 )
        ROS_ERROR_STREAM( "Error deleting file file_1" );

    if ( remove(file_2.c_str()) != 0 )
        ROS_ERROR_STREAM( "Error deleting file file_2" );
}

void setTestName(int lanes, double sigma1, double P1, double P2, double sigma2, double weight)
{
    int precision_naming = 8;
    stringstream stream;
    stream << fixed << setprecision(precision_naming) << sigma1;
    string s_sigma1 = stream.str();
    stream.str("");
    stream.clear();
    stream << fixed << setprecision(precision_naming) << P1;
    string s_P1 = stream.str();
    stream.str("");
    stream.clear();
    stream << fixed << setprecision(precision_naming) << P2;
    string s_P2 = stream.str();
    stream.str("");
    stream.clear();
    stream << fixed << setprecision(precision_naming) << sigma2;
    string s_sigma2 = stream.str();
    stream.str("");
    stream.clear();
    stream << fixed << setprecision(precision_naming) << lanes;
    string s_lanes = stream.str();
    stream.str("");
    stream.clear();
    stream << fixed << setprecision(precision_naming) << weight;
    string s_weight = stream.str();
    stream.str("");
    stream.clear();

    if (sigma2 < 0)
        testname = s_weight + "+" + s_lanes + "+" + s_sigma1 + "+" + s_P1 + "+" + s_P2;
    else
        testname = s_weight + "+" + s_lanes + "+" + s_sigma1 + "+" + s_sigma2 + "+" + s_P1 + "+" + s_P2;

    ROS_INFO_STREAM("Multiprocess: " << multiprocess << "\tTESTNAME: "  << testname);
    ROS_INFO_STREAM("Active BAGFILE: " << BAGFILE);
    ROS_INFO_STREAM("Active GTFILE:  " << GTFILE);
}

void setupEnv(double sigma1, double sigma2, double P1, double P2, int pluscorsie, int lanes_number, double av_weight)
{
    howManyLanes = lanes_number;
    plus_corsie_continue = pluscorsie;
    average_weight = av_weight;

    setTestName(lanes_number, sigma1, P1, P2, sigma2, av_weight);

#ifdef MAKETRANSITIONSMATRICES
    ROS_INFO_STREAM("Generate Transition Matrices ACTIVE");
    makeTransitionMatrixS2(sigma1, sigma2, P1, P2);
#else
    ROS_INFO_STREAM("Transition Matrices HARDCODED");
    setStateTransitionMatrix();
#endif

    resetMegavariabile(howManyLanes);
    resetSensor(howManyLanes);
}

/// END

void oneShot(rosbag::View &view)
{
    unsigned int counter = 0;
    setupEnv(1.14261, 0.493907, 0.0454398, 0.15849, 7, 4, 0.6);
    foreach (rosbag::MessageInstance const messageInstance, view)
    {
        ROS_INFO_STREAM_ONCE("Parsing bagfile ... ");
        ROS_DEBUG_STREAM("READ " << counter);
        counter++;

        // FOR THE FULL A4-5FULL GT, SKIP SOME VALUES AND FINISH BEFORE THE EAST-MILANO BARRIER
        if (counter < 239)
            continue;
        if (counter > 10190)
            break;

        road_layout_estimation::msg_lines::ConstPtr msg_lines;
        msg_lines = messageInstance.instantiate<road_layout_estimation::msg_lines>();
        ROS_ASSERT(msg_lines != NULL);

        // this will create the files needed in the evaluate() routine
        executeTest(*msg_lines);
    }
    ROS_INFO_STREAM("Evaluating results, processed " << counter << " ");
    evaluate(false);
    ROS_INFO_STREAM("End.");
}

void randomSearch(rosbag::View &view)
{
    std::default_random_engine generator(std::random_device{}());
    std::uniform_real_distribution<double>  gen_sigma1(0.0, 3.0);
    std::uniform_real_distribution<double>  gen_sigma2(0.0, 3.0);
    std::uniform_real_distribution<double>  gen_P1(0.0, 1.0);
    std::uniform_real_distribution<double>  gen_P2(0.0, 1.0);
    std::uniform_int_distribution<int>      gen_pluscorsie(0, 9);
    std::uniform_real_distribution<double>  gen_ave_weight(0.0, 1.0);

    double sigma1, sigma2, P1, P2, ave_weight;
    int pluscorsie, lanes_number;
    double fitness = 0.0;

    while (ros::ok())
    {
        // Generate Parameters.
        lanes_number = 4; // this is fixed, same lanes always.
        sigma1 = gen_sigma1(generator);
        sigma2 = gen_sigma1(generator);
        P1 = gen_P1(generator);
        P2 = gen_P2(generator);
        pluscorsie = gen_pluscorsie(generator);
        ave_weight = 0.6; //gen_ave_weight(generator);

        setupEnv( sigma1,  sigma2,  P1,  P2,  pluscorsie, lanes_number, ave_weight);

        unsigned int counter = 0;
        foreach (rosbag::MessageInstance const messageInstance, view)
        {
            ROS_INFO_STREAM_ONCE("Parsing bagfile ... ");
            ROS_DEBUG_STREAM("READ " << counter);
            counter++;

            // FOR THE FULL A4-5FULL GT, SKIP SOME VALUES AND FINISH BEFORE THE EAST-MILANO BARRIER
            if (counter < 239)
                continue;
            if (counter > 10190)
                break;

            road_layout_estimation::msg_lines::ConstPtr msg_lines;
            msg_lines = messageInstance.instantiate<road_layout_estimation::msg_lines>();
            ROS_ASSERT(msg_lines != NULL);

            // this will create the files needed in the evaluate() routine
            executeTest(*msg_lines);
        }

        ROS_INFO_STREAM("Evaluating results, processed " << counter << " ");
        double tmp_fitness = evaluate();

        if (tmp_fitness > fitness)
        {
            counter = 0;
            fitness = tmp_fitness;
            ROS_INFO_STREAM("NEW CONFIGURATION FOUND!");

            // save configuration
            ofstream myfile;
            char separator = ',';
            ROS_DEBUG_STREAM("Saving results in: " << SAVEPATH << "RANDOMSEARCH.txt");
            myfile.open (SAVEPATH "RANDOMSEARCH." + multiprocess + ".txt", ios::app);
            myfile << fitness << separator << sigma1 << separator << sigma2 << separator << P1 << separator << P2 << separator << pluscorsie << separator << lanes_number << separator << ave_weight << endl;
            myfile.close();

            ROS_WARN_STREAM("TESTING CONFIG:\t" << sigma1 << ";" <<  sigma2 << ";" <<  P1 << ";" <<  P2 << ";" <<  pluscorsie << ";" << lanes_number << ";" << ave_weight);
            setupEnv( sigma1,  sigma2,  P1,  P2,  pluscorsie, lanes_number, ave_weight);
            foreach (rosbag::MessageInstance const messageInstance, view)
            {
                counter++;
                // FOR THE FULL A4-5FULL GT, SKIP SOME VALUES AND FINISH BEFORE THE EAST-MILANO BARRIER
                if (counter < 239)
                    continue;
                if (counter > 10190)
                    break;


                road_layout_estimation::msg_lines::ConstPtr msg_lines;
                msg_lines = messageInstance.instantiate<road_layout_estimation::msg_lines>();
                ROS_ASSERT(msg_lines != NULL);
                executeTest(*msg_lines);
            }
            double check_fitness = evaluate();
            ROS_WARN_STREAM("fitness 1:\t" << fitness);
            ROS_WARN_STREAM("fitness 2:\t" << check_fitness);
        }

        ROS_INFO_STREAM("================================================");
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane");
    ros::NodeHandle n;

    //if ( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    //    ros::console::notifyLoggerLevelsChanged();
    //else
    //    std::cout << "Error while setting the logger level!" << std::endl;

    if (argc == 2)
    {
        multiprocess = argv[1];
        cout << multiprocess << endl;
    }


#if SYNCMODE

    ROS_INFO_STREAM("Running " << ros::this_node::getName() << " in SYNCMOD");
    ros::Subscriber sub = n.subscribe("/isis_line_detector/lines", 1000, executeTest);
    sync_publisher = new ros::Publisher;
    *sync_publisher = n.advertise<std_msgs::Bool>("/sync", 1);
    ros::spin();

#else
    ROS_INFO_STREAM("Running " << ros::this_node::getName() << " in BAGMODE");
    rosbag::Bag bag;
    ROS_INFO_STREAM("Opening bagfile...");
    bag.open(BAGFILE, rosbag::bagmode::Read);
    ROS_INFO_STREAM("Opened, OK!");

    std::vector<std::string> topics;
    topics.push_back(std::string("/isis_line_detector/lines"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    unsigned int counter = 0;

    // One Shot
    oneShot(view);
    return 1;

    // Random Search
    //randomSearch(view);
    //return 1;

    // Increase number of plusCorsie
    for (int increasePlusCorsie = 1; increasePlusCorsie <= 4; increasePlusCorsie++)
    {
        //setupEnv(0.72f, 0.72f, 0.9f, 0.2f, increasePlusCorsie, 4, 0.6);
        //setupEnv(0.116786,0.391961,0.0455318,0.161657,9,4);
        //setupEnv(0.0521648,0.0580163,0.170258,0.42129,6,4);
        //setupEnv(0.188789,0.315906,0.447387,0.257911,7,4,0.6); // BEST DOMENICA i-5
        //setupEnv(0.228095,0.766402,0.675454,0.872998,1,4,0.6);
        //setupEnv(0.347901,0.367047,0.334395,0.692325,9,4,0.219284); // BEST 48CORE before Maurino...
        setupEnv(1.14261, 0.493907, 0.0454398, 0.15849, 7, 4, 0.6);

        counter = 0;
        foreach (rosbag::MessageInstance const messageInstance, view)
        {
            ROS_INFO_STREAM_ONCE("Parsing bagfile ... ");
            ROS_DEBUG_STREAM("READ " << counter);
            counter++;

            // FOR THE FULL A4-5FULL GT, SKIP SOME VALUES AND FINISH BEFORE THE EAST-MILANO BARRIER
            if (counter < 239)
                continue;
            if (counter > 10190)
                break;

            road_layout_estimation::msg_lines::ConstPtr msg_lines;
            msg_lines = messageInstance.instantiate<road_layout_estimation::msg_lines>();
            ROS_ASSERT(msg_lines != NULL);

            // this will create the files needed in the evaluate() routine
            executeTest(*msg_lines);
        }
        ROS_INFO_STREAM("Evaluating results, processed " << counter << " ");
        evaluate();
        ROS_INFO_STREAM("================================================");
    }

#endif


    return 1;
}

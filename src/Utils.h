/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Author:    Dario Limongi                                              *
 *   Email:     dario.limongi@gmail.com                                    *
 *   Date:      22/01/2014                                                 *
 *                                                                         *
 ***************************************************************************/

#ifndef UTILS_H
#define UTILS_H

#include <stdlib.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include "particle/State6DOF.h"

using namespace Eigen;

class Utils
{

public:

    struct Coordinates
    {
        float latitude;
        float longitude;
        float altitude;
    };

    static geometry_msgs::Point latlon_converter(double lat, double lngd);

    static Coordinates xy2latlon(double x, double y);

    static Coordinates ecef2lla(double x, double y, double z);

    static geometry_msgs::Point lla2ecef(double lat, double lon, double alt);

    static nav_msgs::Odometry addNoiseAndCovToOdom(const nav_msgs::Odometry & step, double pos_err, double or_err, double lin_err, double ang_err);

    static geometry_msgs::Twist getSpeed(ros::Time& prev_time, ros::Time& curr_time, const tf::Transform & temp_t, const tf::Transform & t);

    static geometry_msgs::Twist getSpeedFrom2PoseStamped(const geometry_msgs::PoseStamped & pose_prec, const geometry_msgs::PoseStamped & pose_t);

    static void sendTfFromPoseStamped(const geometry_msgs::PoseStamped &pose, tf::TransformBroadcaster *tfb);

    static void printOdomAngleAxisToCout(const nav_msgs::Odometry& msg);

    static void printOdomMsgToCout(const nav_msgs::Odometry& msg);

    static void printPoseMsgToCout(const geometry_msgs::PoseStamped &pose);

    static int linesFromLanes(int number_of_lanes);

    static int lanesFromLines(int goodLines);

    ///
    /// \brief addOffsetToVectorXd
    /// \param pose
    /// \param position_err
    /// \param orientation_err
    /// \param speed_err
    /// \return Adds a random noise to a VectorXd 12x1 representing particle's pose
    ///
    static VectorXd addOffsetToVectorXd(const VectorXd& pose, double position_err, double orientation_err, double speed_err);

    ///
    /// \brief getOdomFromPoseAndSigma
    /// \param pose
    /// \param sigma
    /// \return
    ///
    /// Returns a message of type nav_msgs::Odometry given
    /// a VectorXd as parameter
    ///
    static nav_msgs::Odometry getOdomFromPoseAndSigma(const VectorXd& pose, const MatrixXd& sigma);

    ///
    /// \brief getPoseVectorFromOdom
    /// \param msg
    /// \return
    /// Return a 12x1 VectorXd representing the pose given
    /// a message of type nav_msgs::Odometryfrom
    ///
    static VectorXd getPoseVectorFromOdom(const nav_msgs::Odometry& msg);

    ///
    /// \brief getCovFromOdom
    /// \param msg
    /// \return
    ///
    static MatrixXd getCovFromOdom(const nav_msgs::Odometry& msg);

    ///
    /// \brief getPoseFromVector
    /// \param msg
    /// \return
    ///
    static geometry_msgs::Pose getPoseFromVector(const VectorXd& msg);

    ///
    /// \brief getNoise
    /// \param err
    /// \return a random number between -err and err
    ///
    static double getNoise(double err);

    ///
    /// \brief box_muller
    /// \param m
    /// \param s
    /// \return a random number sampled from normal distribution with mean m and std s
    ///
    static double box_muller(double m, double s);

    ///
    /// \brief getRotationMatrix
    /// \param q
    /// \return
    ///
    static Eigen::Matrix3d getRotationMatrix(Eigen::Vector4d q);

    ///
    /// \brief getCameraCenterAfterRotation
    /// \param c
    /// \param r
    /// \return
    ///
    static Eigen::Vector3d getCameraCenterAfterRotation(const double c[3], Eigen::Matrix3d r);

    /**
     * @brief Utils::rad2deg
     * @param alpha
     * @return
     *
     * from PCL library
     */
    static inline double rad2deg (double alpha)
    {
        return (alpha * 57.29578);
    }

    /**
     * @brief Utils::deg2rad
     * @param alpha
     * @return
     *
     * from PCL library
     */
    static inline double deg2rad (double alpha)
    {
        return (alpha * 0.017453293);
    }

    ///
    /// \brief normalize_angle
    /// \param z the angle to normalize
    /// \return an equivalent angle in the range (-Pi, Pi]
    ///
    /// Computes the normalized value of an angle, which is the equivalent angle in the range ( -Pi, Pi ].
    ///

    /**
    * @brief Utils::normalize_angle
    * @param z the angle to normalize
    * @return an equivalent angle in the range (-Pi, Pi]
    *
    * Computes the normalized value of an angle, which is the equivalent angle in the range ( -Pi, Pi ].
    */
    static inline double normalize_angle(double z)
    {
        return atan2(sin(z), cos(z));
    }

    ///
    /// \brief angle_diff
    /// \param a the angle of one vector
    /// \param b the angle of the other vector
    /// \return the angle (in radians) between the two vectors (in range [0, Pi] )
    ///
    /// Computes the unoriented smallest difference between two angles.
    ///
    static double angle_diff(double a, double b);

    static tf::Stamped<tf::Pose> toGlobalFrame(Vector3d p_state);
};



#endif // UTILS_H

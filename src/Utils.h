#ifndef UTILS_H
#define UTILS_H

#include <stdlib.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

using namespace Eigen;

class Utils{
public:

    struct Coordinates
    {
        float latitude;
        float longitude;
        float altitude;
    };

    static geometry_msgs::Point latlon_converter(double lat, double lngd);

    static Coordinates ecef2lla(double x, double y, double z);

    static geometry_msgs::Point lla2ecef(double lat, double lon, double alt);

    static nav_msgs::Odometry addNoiseAndCovToOdom(const nav_msgs::Odometry & step, double pos_err, double or_err, double lin_err, double ang_err);

    static geometry_msgs::Twist getSpeed(ros::Time& prev_time, ros::Time& curr_time, const tf::Transform & temp_t, const tf::Transform & t);

    static geometry_msgs::Twist getSpeedFrom2PoseStamped(const geometry_msgs::PoseStamped & pose_prec, const geometry_msgs::PoseStamped & pose_t);

    static void sendTfFromPoseStamped(const geometry_msgs::PoseStamped &pose, tf::TransformBroadcaster *tfb);

    static void printOdomMsgToCout(const nav_msgs::Odometry& msg);

    /**
     * @param pose
     * @param position_offset
     * @param orientation_offset
     * @param speed_offset
     * @return Adds a random noise to a VectorXd 12x1 representing particle's pose
     */
    static VectorXd addOffsetToVectorXd(const VectorXd& pose, double position_err, double orientation_err, double speed_err);

    /**
     * Returns a message of type nav_msgs::Odometry given
     * a VectorXd as parameter
     * @param pose
     * @return
     */
    static nav_msgs::Odometry getOdomFromPoseAndSigma(const VectorXd& pose, const MatrixXd& sigma);

    /**
     * Return a 12x1 VectorXd representing the pose given
     * a message of type nav_msgs::Odometryfrom
     * @param msg
     * @return
     */
    static VectorXd getPoseVectorFromOdom(const nav_msgs::Odometry& msg);

    /**
     * @brief getCovFromOdom
     * @param msg
     * @return
     */
    static MatrixXd getCovFromOdom(const nav_msgs::Odometry& msg);

    /**
     * @brief getPoseFromVector
     * @param msg
     * @return
     */
    static geometry_msgs::Pose getPoseFromVector(const VectorXd& msg);

    /**
     * @param err
     * @return a random number between -err and err
     */
    static double getNoise(double err);

    /**
     * @brief getRotationMatrix
     * @param q
     * @return
     */
    static Eigen::Matrix3d getRotationMatrix(Eigen::Vector4d q);

    /**
     * @brief getCameraCenterAfterRotation
     * @param c
     * @param r
     * @return
     */
    static Eigen::Vector3d getCameraCenterAfterRotation(const double c[3], Eigen::Matrix3d r);

    static double rad2deg (double alpha);
    static double deg2rad (double alpha);

    /**
     * Computes the normalized value of an angle, which is the equivalent angle in the range ( -Pi, Pi ].
     * @param z	the angle to normalize
     * @return an equivalent angle in the range (-Pi, Pi]
     */
    static double normalize_angle(double z);

    /**
     * Computes the unoriented smallest difference between two angles.
     *
     * @param a the angle of one vector
     * @param b the angle of the other vector
     * @return the angle (in radians) between the two vectors (in range [0, Pi] )
     */
    static double angle_diff(double a, double b);
};



#endif // UTILS_H

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

#include "Utils.h"

#include <stdlib.h>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "Utils.h"
#include <boost/assign.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// Conversion between geographic and UTM coordinates
// Adapted from:
// http://www.uwgb.edu/dutchs/UsefulData/ConvertUTMNoOZ.HTM
geometry_msgs::Point Utils::latlon_converter(double lat, double lngd){

    // WGS 84 datum
    double eqRad = 6378137.0;
    double flat = 298.2572236;

    // constants used in calculations:
    double a = eqRad;           // equatorial radius in meters
    double f = 1.0 / flat;        // polar flattening
    double b = a * (1.0 - f);     // polar radius
    double e = sqrt(1.0 - (pow(b,2) / pow(a,2)));  // eccentricity
    double k0 = 0.9996;
    double drad = M_PI / 180.0;

    double phi = lat * drad;   // convert latitude to radians
    double utmz = 1.0 + floor((lngd + 180.0) / 6.0); // longitude to utm zone
    double zcm = 3.0 + 6.0 * (utmz - 1.0) - 180.0;     // central meridian of a zone
    double esq = (1.0 - (b / a) * (b / a));
    double e0sq = e * e / (1.0 - e * e);
    double M = 0.0;
    double M0 = 0.0;
    double N = a / sqrt(1.0 - pow(e * sin(phi), 2));
    double T = pow(tan(phi), 2);
    double C = e0sq * pow(cos(phi), 2);
    double A = (lngd - zcm) * drad * cos(phi);

    // calculate M (USGS style)
    M = phi * (1.0 - esq * (1.0 / 4.0 + esq * (3.0 / 64.0 + 5.0 * esq / 256.0)));
    M = M - sin(2.0 * phi) * (esq * (3.0 / 8.0 + esq * (3.0 / 32.0 + 45.0 * esq / 1024.0)));
    M = M + sin(4.0 * phi) * (esq * esq * (15.0 / 256.0 + esq * 45.0 / 1024.0));
    M = M - sin(6.0 * phi) * (esq * esq * esq * (35.0 / 3072.0));
    M = M * a; // Arc length along standard meridian

    // now we are ready to calculate the UTM values...
    // first the easting (relative to CM)
    double x = k0 * N * A * (1.0 + A * A * ((1.0 - T + C) / 6.0 + A * A * (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * e0sq) / 120.0));
    x = x + 500000.0; // standard easting

    // now the northing (from the equator)
    double y = k0 * (M - M0 + N * tan(phi) * (A * A * (1.0 / 2.0 + A * A * ((5.0 - T + 9.0 * C + 4.0 * C * C) / 24.0 + A * A * (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * e0sq) / 720.0))));
    if (y < 0){
        y = 10000000.0 + y; // add in false northing if south of the equator
    }
    double easting = round(10.0*x) / 10.0;
    double northing = round(10.0*y) / 10.0;

    geometry_msgs::Point coords;
    coords.x = easting;
    coords.y = northing;
    coords.z = 0;

    return coords;
 }

/**
 * @brief deg_to_radians
 * @param x
 * @return
 */
double deg_to_radians(double x)
{
    double RADIANS = 57.2957795; 		/* degrees/radian */
    return (x/RADIANS);
}

/**
 * @brief meters_deglon
 * @param x
 * @return
 */
double meters_deglon(double x)
{
    double d2r = deg_to_radians(x);
    return((111415.13f * cos(d2r))- (94.55f * cos(3.0f*d2r)) + (0.12f * cos(5.0f*d2r)));
}

/**
 * @brief meters_deglat
 * @param x
 * @return
 */
double meters_deglat(double x)
{
    double d2r = deg_to_radians(x);
    return(111132.09f - (566.05f * cos(2.0f*d2r))+ (1.20f * cos(4.0f*d2r)) - (0.002f * cos(6.0f*d2r)));
}

/**
 * ECEF2LLA - convert earth-centered earth-fixed (ECEF)
 *            cartesian coordinates to latitude, longitude,
 *            and altitude
 *
 * USAGE:
 * [lat,lon,alt] = ecef2lla(x,y,z)
 *
 * lat = geodetic latitude (radians)
 * lon = longitude (radians)
 * alt = height above WGS84 ellipsoid (m)
 * x = ECEF X-coordinate (m)
 * y = ECEF Y-coordinate (m)
 * z = ECEF Z-coordinate (m)
 *
 * Notes: (1) This function assumes the WGS84 model.
 *        (2) Latitude is customary geodetic (not geocentric).
 *        (3) Inputs may be scalars, vectors, or matrices of the same
 *            size and shape. Outputs will have that same size and shape.
 *        (4) Tested but no warranty; use at your own risk.
 *        (5) Michael Kleder, April 2006
 * (C++ porting by Dario Limongi)
 */
Utils::Coordinates Utils::ecef2lla(double x, double y, double z)
{
    Utils::Coordinates coords;

    // WGS84 ellipsoid constants:
    double a = 6378137;            // (m) at the equator
    double e = 0.081819190842622;  // eccentricity

    // calculations:
    double b   = sqrt( (a*a) * (1 - e*e ));
    double ep  = sqrt( (a*a - b*b ) / (b*b));
    double p   = sqrt( x*x + y*y );
    double th  = atan2(a*z,b*p);
    double lon = atan2(y,x);
    double lat = atan2( (z + ep*ep * b * sin(th)*sin(th)*sin(th)), (p - e*e *a * cos(th)*cos(th)*cos(th)) );
    double N   = a / sqrt( 1 - e*e * sin(lat) * sin(lat));
    double alt = p / cos(lat) - N;

    // return lon in range [0,2*pi)
    lon = lon - (M_PI*2) * floor( lon / (M_PI*2) );

    // correct for numerical instability in altitude near exact poles:
    // (after this correction, error is about 2 millimeters, which is about
    // the same as the numerical precision of the overall function)
//    double k = abs(x)<1 & abs(y)<1;
//    double alt(k) = abs(z(k))-b;

    coords.longitude = lon;
    coords.latitude = lat;
    coords.altitude = alt;

    return coords;
}

/**
 * LLA2ECEF - convert latitude, longitude, and altitude to
 *             earth-centered, earth-fixed (ECEF) cartesian
 *
 * USAGE:
 * [x,y,z] = lla2ecef(lat,lon,alt)
 *
 * x = ECEF X-coordinate (m)
 * y = ECEF Y-coordinate (m)
 * z = ECEF Z-coordinate (m)
 * lat = geodetic latitude (radians)
 * lon = longitude (radians)
 * alt = height above WGS84 ellipsoid (m)
 *
 * Notes: This function assumes the WGS84 model.
 *        Latitude is customary geodetic (not geocentric).
 *
 * Source: "Department of Defense World Geodetic System 1984"
 *         Page 4-4
 *         National Imagery and Mapping Agency
 *         Last updated June, 2004
 *         NIMA TR8350.2
 *
 * Michael Kleder, July 2005
 * (C++ porting by Dario Limongi)
 */
geometry_msgs::Point Utils::lla2ecef(double lat, double lon, double alt)
{
    geometry_msgs::Point point;

    // WGS84 ellipsoid constants:
    double a = 6378137;            // (m) at the equator
    double e = 0.081819190842622;  // eccentricity

    // intermediate calculation
    // (prime vertical radius of curvature)
    double N = a / sqrt(1 - (e*e) * (sin(lat)*sin(lat)));

    // results:
    point.x = (N + alt) * cos(lat) * cos(lon);
    point.y = (N + alt) * cos(lat) * sin(lon);
    point.z = ((1-(e*e)) * N + alt) * sin(lat);

    return point;
}

/**
 * Adds a random noise to odometry SINGLE step
 * @param steps
 * @return noisy steps
 */
nav_msgs::Odometry Utils::addNoiseAndCovToOdom(const nav_msgs::Odometry & step, double pos_err, double or_err, double lin_err, double ang_err)
{

    nav_msgs::Odometry noisy_step = step;
    //set covariance
    noisy_step.pose.covariance =  boost::assign::list_of  (pos_err) (0)   (0)  (0)  (0)  (0)
                                                            (0)  (pos_err)  (0)  (0)  (0)  (0)
                                                            (0)   (0)  (pos_err) (0)  (0)  (0)
                                                            (0)   (0)   (0) (or_err) (0)  (0)
                                                            (0)   (0)   (0)  (0) (or_err) (0)
                                                            (0)   (0)   (0)  (0)  (0)  (or_err) ;

    noisy_step.twist.covariance =  boost::assign::list_of  (lin_err) (0)   (0)  (0)  (0)  (0)
                                                           (0)  (lin_err)  (0)  (0)  (0)  (0)
                                                           (0)   (0)  (lin_err) (0)  (0)  (0)
                                                           (0)   (0)   (0) (ang_err) (0)  (0)
                                                           (0)   (0)   (0)  (0) (ang_err) (0)
                                                           (0)   (0)   (0)  (0)  (0)  (ang_err) ;
    // adds noise to the step
    noisy_step.pose.pose.position.x += Utils::getNoise(noisy_step.pose.covariance.elems[0]);
    noisy_step.pose.pose.position.y += Utils::getNoise(noisy_step.pose.covariance.elems[7]);
    noisy_step.pose.pose.position.z += Utils::getNoise(noisy_step.pose.covariance.elems[14]);
    noisy_step.pose.pose.orientation.x += Utils::getNoise(noisy_step.pose.covariance.elems[21]);
    noisy_step.pose.pose.orientation.y += Utils::getNoise(noisy_step.pose.covariance.elems[28]);
    noisy_step.pose.pose.orientation.z += Utils::getNoise(noisy_step.pose.covariance.elems[35]);

    //normalize angles
    noisy_step.pose.pose.orientation.x = Utils::normalize_angle(noisy_step.pose.pose.orientation.x);
    noisy_step.pose.pose.orientation.y = Utils::normalize_angle(noisy_step.pose.pose.orientation.y);
    noisy_step.pose.pose.orientation.z = Utils::normalize_angle(noisy_step.pose.pose.orientation.z);

    noisy_step.twist.twist.angular.x += Utils::getNoise(noisy_step.twist.covariance.elems[0]);
    noisy_step.twist.twist.angular.y += Utils::getNoise(noisy_step.twist.covariance.elems[7]);
    noisy_step.twist.twist.angular.z += Utils::getNoise(noisy_step.twist.covariance.elems[14]);
    noisy_step.twist.twist.linear.x += Utils::getNoise(noisy_step.twist.covariance.elems[21]);
    noisy_step.twist.twist.linear.y += Utils::getNoise(noisy_step.twist.covariance.elems[28]);
    noisy_step.twist.twist.linear.z += Utils::getNoise(noisy_step.twist.covariance.elems[35]);

    return noisy_step;
}

geometry_msgs::Twist Utils::getSpeed(ros::Time& prev_time, ros::Time& curr_time, const tf::Transform & temp_t, const tf::Transform & t){

    double rate = curr_time.toSec() - prev_time.toSec();
    geometry_msgs::Twist speed;

    speed.linear.x = (t.getOrigin().getX() - temp_t.getOrigin().getX()) / rate;
    speed.linear.y = (t.getOrigin().getY() - temp_t.getOrigin().getY()) / rate;
    speed.linear.z = (t.getOrigin().getZ() - temp_t.getOrigin().getZ()) / rate;

    // Quaternion to RPY (step_prec)
    tf::Matrix3x3 m(temp_t.getRotation());
    double roll_prec; double pitch_prec; double yaw_prec;
    m.getRPY(roll_prec, pitch_prec, yaw_prec);

    // Quaternion to RPY (step_t)
    tf::Matrix3x3 m_t(t.getRotation());
    double roll_t; double pitch_t; double yaw_t;
    m_t.getRPY(roll_t, pitch_t, yaw_t);

    speed.angular.x = ( Utils::angle_diff(roll_t, roll_prec) ) / rate;
    speed.angular.y = ( Utils::angle_diff(pitch_t, pitch_prec) ) / rate;
    speed.angular.z = ( Utils::angle_diff(yaw_t, yaw_prec) ) / rate;

    return speed;
}

/**
 * @brief getSpeed
 * @param pose_prec
 * @param pose_t
 * @return current_speed
 */
geometry_msgs::Twist Utils::getSpeedFrom2PoseStamped(const geometry_msgs::PoseStamped & pose_prec, const geometry_msgs::PoseStamped & pose_t){
    geometry_msgs::Twist speed;
    double rate = pose_t.header.stamp.toSec() - pose_prec.header.stamp.toSec();

    if(rate == 0){
        speed.linear.x = 0;
        speed.linear.y = 0;
        speed.linear.z = 0;
        speed.angular.x = 0;
        speed.angular.y = 0;
        speed.angular.z = 0;

        return speed;
    }

    // calculate linear speeds
    speed.linear.x = (pose_t.pose.position.x - pose_prec.pose.position.x) / rate;
    speed.linear.y = (pose_t.pose.position.y - pose_prec.pose.position.y) / rate;
    speed.linear.z = (pose_t.pose.position.z - pose_prec.pose.position.z) / rate;

    // Quaternion to RPY (step_prec)
    tf::Quaternion q1; tf::Quaternion q2;
    tf::quaternionMsgToTF(pose_prec.pose.orientation, q1);
    tf::Matrix3x3 m(q1);
    double roll_prec; double pitch_prec; double yaw_prec;
    m.getRPY(roll_prec, pitch_prec, yaw_prec);

    // Quaternion to RPY (step_t)
    tf::quaternionMsgToTF(pose_t.pose.orientation,q2);
    tf::Matrix3x3 m_t(q2);
    double roll_t; double pitch_t; double yaw_t;
    m_t.getRPY(roll_t, pitch_t, yaw_t);

    // calculate angular speeds
    speed.angular.x = ( Utils::angle_diff(roll_t, roll_prec) ) / rate;
    speed.angular.y = ( Utils::angle_diff(pitch_t, pitch_prec) ) / rate;
    speed.angular.z = ( Utils::angle_diff(yaw_t, yaw_prec) ) / rate;

    return speed;
}

void Utils::sendTfFromPoseStamped(const geometry_msgs::PoseStamped& pose, tf::TransformBroadcaster* tfb)
{
    tf::Transform t1;
    tf::Point point;
    tf::Quaternion q;
    tf::pointMsgToTF(pose.pose.position, point);
    tf::quaternionMsgToTF(pose.pose.orientation, q);
    t1.setOrigin(point);
    t1.setRotation(q);
    tfb->sendTransform(tf::StampedTransform(t1, pose.header.stamp, "robot_frame", "odom_frame"));
}

void Utils::printOdomMsgToCout(const nav_msgs::Odometry& msg)
{
    std::cout << " Position:" << std::endl;
    std::cout << "  x: " << msg.pose.pose.position.x << std::endl;
    std::cout << "  y: " << msg.pose.pose.position.y << std::endl;
    std::cout << "  z: " << msg.pose.pose.position.z << std::endl;
    std::cout << " Orientation quaternion: " << std::endl;
    std::cout << "  w: " << msg.pose.pose.orientation.w << std::endl;
    std::cout << "  x: " << msg.pose.pose.orientation.x << std::endl;
    std::cout << "  y: " << msg.pose.pose.orientation.y << std::endl;
    std::cout << "  z: " << msg.pose.pose.orientation.z << std::endl;
    std::cout << " Linear speed: " << std::endl;
    std::cout << "  x: " << msg.twist.twist.linear.x << std::endl;
    std::cout << "  y: " << msg.twist.twist.linear.y << std::endl;
    std::cout << "  z: " << msg.twist.twist.linear.z << std::endl;
    std::cout << " Angular speed: " << std::endl;
    std::cout << "  x: " << msg.twist.twist.angular.x << std::endl;
    std::cout << "  y: " << msg.twist.twist.angular.y << std::endl;
    std::cout << "  z: " << msg.twist.twist.angular.z << std::endl;
}

void Utils::printOdomAngleAxisToCout(const nav_msgs::Odometry& msg)
{
    State6DOF tmp(msg);

    std::cout << "[measure_t ]" << std::endl;
    std::cout << "       pose: " << tmp._pose.transpose() << std::endl << "orientation: " << tmp._rotation.angle() << " " << tmp._rotation.axis().transpose() << std::endl;
    std::cout << "     linear: " << tmp._translational_velocity.transpose() << std::endl << "    angular: " << tmp._rotational_velocity.angle() << " " << tmp._rotational_velocity.axis().transpose() << std::endl << std::endl;

    std::cout << std::endl;
}


void Utils::printPoseMsgToCout(const geometry_msgs::PoseStamped &pose){
    std::cout << " Position:" << std::endl;
    std::cout << "  x: " << pose.pose.position.x << std::endl;
    std::cout << "  y: " << pose.pose.position.y << std::endl;
    std::cout << "  z: " << pose.pose.position.z << std::endl;
    std::cout << " Orientation quaternion: " << std::endl;
    std::cout << "  w: " << pose.pose.orientation.w << std::endl;
    std::cout << "  x: " << pose.pose.orientation.x << std::endl;
    std::cout << "  y: " << pose.pose.orientation.y << std::endl;
    std::cout << "  z: " << pose.pose.orientation.z << std::endl;
}

/**
 * @param pose
 * @param position_offset
 * @param orientation_offset
 * @param speed_offset
 * @return Adds a random noise to a VectorXd 12x1 representing particle's pose
 */
VectorXd Utils::addOffsetToVectorXd(const VectorXd& pose, double position_offset, double orientation_offset, double speed_offset)
{
    srand(time(0));
    VectorXd vec = pose;
    vec(0) += Utils::getNoise(position_offset);
    vec(1) += Utils::getNoise(position_offset);
    vec(2) += Utils::getNoise(position_offset);

    vec(3) += Utils::getNoise(orientation_offset); vec(3) = atan2(sin(vec(3)),cos(vec(3)));
    vec(4) += Utils::getNoise(orientation_offset); vec(4) = atan2(sin(vec(4)),cos(vec(4)));
    vec(5) += Utils::getNoise(orientation_offset); vec(5) = atan2(sin(vec(5)),cos(vec(5)));

    vec(6) += Utils::getNoise(speed_offset);
    vec(7) += Utils::getNoise(speed_offset);
    vec(8) += Utils::getNoise(speed_offset);
    vec(9) += Utils::getNoise(speed_offset);
    vec(10) += Utils::getNoise(speed_offset);
    vec(11) += Utils::getNoise(speed_offset);

    return vec;
}


VectorXd Utils::getPoseVectorFromOdom(const nav_msgs::Odometry& msg)
{
    VectorXd pose = VectorXd::Zero(12);

    //state
    pose(0) = msg.pose.pose.position.x;
    pose(1) = msg.pose.pose.position.y;
    pose(2) = msg.pose.pose.position.z;

    //orientation
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, q);
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pose(3) = roll;
    pose(4) = pitch;
    pose(5) = yaw;

    //speed
    pose(6) = msg.twist.twist.linear.x;
    pose(7) = msg.twist.twist.linear.y;
    pose(8) = msg.twist.twist.linear.z;
    pose(9) = msg.twist.twist.angular.x;
    pose(10) = msg.twist.twist.angular.y;
    pose(11) = msg.twist.twist.angular.z;

    return pose;
}

MatrixXd Utils::getCovFromOdom(const nav_msgs::Odometry& msg){
    MatrixXd pose_cov = MatrixXd::Zero(6,6);
    MatrixXd twist_cov = MatrixXd::Zero(6,6);
    MatrixXd cov = MatrixXd::Zero(12,12);

    // build cov array from boost::matrix
    double pose_cov_array [36];
    for(int i=0; i<36; i++){
        pose_cov_array[i] = msg.pose.covariance.elems[i];
    }

    // build cov array from boost::matrix
    double twist_cov_array [36];
    for(int i=0; i<36; i++){
        twist_cov_array[i] = msg.twist.covariance.elems[i];
    }

    // build the two 6x6 cov matrix from arrays
    int counter = 0;
    for(int row=0; row<6; row++)
    {
        for(int column=0; column<6; column++)
        {
            pose_cov(row,column) = pose_cov_array[counter];
            twist_cov(row,column) = twist_cov_array[counter];
            counter++;
        }
    }

    // join the two matrix on top-left and bottom-right corner of the 12x12 cov matrix
    cov.topLeftCorner(6,6) = pose_cov;
    cov.bottomRightCorner(6,6) = twist_cov;

    return cov;
}

nav_msgs::Odometry Utils::getOdomFromPoseAndSigma(const VectorXd& pose, const MatrixXd& sigma){

    // istantiate odom
    nav_msgs::Odometry odom;

    //position
    odom.pose.pose.position.x = pose(0);
    odom.pose.pose.position.y = pose(1);
    odom.pose.pose.position.z = pose(2);

    //orientation
    tf::Quaternion odom_quat = tf::createQuaternionFromRPY(double(pose(3)), double(pose(4)), double(pose(5)));
    odom.pose.pose.orientation.x = odom_quat.getX();
    odom.pose.pose.orientation.y = odom_quat.getY();
    odom.pose.pose.orientation.z = odom_quat.getZ();
    odom.pose.pose.orientation.w = odom_quat.getW();

    //speed
    odom.twist.twist.linear.x = pose(6);
    odom.twist.twist.linear.y = pose(7);
    odom.twist.twist.linear.z = pose(8);
    odom.twist.twist.angular.x = pose(9);
    odom.twist.twist.angular.y = pose(10);
    odom.twist.twist.angular.z = pose(11);

    //covariance
    MatrixXd pose_cov = MatrixXd::Zero(6,6);
    MatrixXd twist_cov = MatrixXd::Zero(6,6);
    pose_cov = sigma.topLeftCorner(6,6);
    twist_cov = sigma.bottomRightCorner(6,6);

    odom.pose.covariance = boost::assign::list_of
            (double(pose_cov(0,0)))   (double(pose_cov(0,1)))  (double(pose_cov(0,2)))   (double(pose_cov(0,3)))   (double(pose_cov(0,4)))  (double(pose_cov(0,5)))
            (double(pose_cov(1,0)))   (double(pose_cov(1,1)))  (double(pose_cov(1,2)))   (double(pose_cov(1,3)))   (double(pose_cov(1,4)))  (double(pose_cov(1,5)))
            (double(pose_cov(2,0)))   (double(pose_cov(2,1)))  (double(pose_cov(2,2)))   (double(pose_cov(2,3)))   (double(pose_cov(2,4)))  (double(pose_cov(2,5)))
            (double(pose_cov(3,0)))   (double(pose_cov(3,1)))  (double(pose_cov(3,2)))   (double(pose_cov(3,3)))   (double(pose_cov(3,4)))  (double(pose_cov(3,5)))
            (double(pose_cov(4,0)))   (double(pose_cov(4,1)))  (double(pose_cov(4,2)))   (double(pose_cov(4,3)))   (double(pose_cov(4,4)))  (double(pose_cov(4,5)))
            (double(pose_cov(5,0)))   (double(pose_cov(5,1)))  (double(pose_cov(5,2)))   (double(pose_cov(5,3)))   (double(pose_cov(5,4)))  (double(pose_cov(5,5)));

    odom.twist.covariance = boost::assign::list_of
            (double(twist_cov(0,0)))   (double(twist_cov(0,1)))  (double(twist_cov(0,2)))   (double(twist_cov(0,3)))   (double(twist_cov(0,4)))  (double(twist_cov(0,5)))
            (double(twist_cov(1,0)))   (double(twist_cov(1,1)))  (double(twist_cov(1,2)))   (double(twist_cov(1,3)))   (double(twist_cov(1,4)))  (double(twist_cov(1,5)))
            (double(twist_cov(2,0)))   (double(twist_cov(2,1)))  (double(twist_cov(2,2)))   (double(twist_cov(2,3)))   (double(twist_cov(2,4)))  (double(twist_cov(2,5)))
            (double(twist_cov(3,0)))   (double(twist_cov(3,1)))  (double(twist_cov(3,2)))   (double(twist_cov(3,3)))   (double(twist_cov(3,4)))  (double(twist_cov(3,5)))
            (double(twist_cov(4,0)))   (double(twist_cov(4,1)))  (double(twist_cov(4,2)))   (double(twist_cov(4,3)))   (double(twist_cov(4,4)))  (double(twist_cov(4,5)))
            (double(twist_cov(5,0)))   (double(twist_cov(5,1)))  (double(twist_cov(5,2)))   (double(twist_cov(5,3)))   (double(twist_cov(5,4)))  (double(twist_cov(5,5)));


    return odom;
}

geometry_msgs::Pose Utils::getPoseFromVector(const VectorXd& msg){
   geometry_msgs::Pose pose;

   //set position
   pose.position.x = msg(0);
   pose.position.y = msg(1);
   pose.position.z = msg(2);

   //normalize orientation
   double roll = atan2(sin(msg(3)),cos(msg(3)));
   double pitch = atan2(sin(msg(4)),cos(msg(4)));
   double yaw = atan2(sin(msg(5)),cos(msg(5)));

   //set orientation
   tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
   tf::quaternionTFToMsg(q, pose.orientation);

   return pose;
}

/**
 * @param err
 * @return a random number between -err and err
 */
double Utils::getNoise(double err){
    srand(time(0));
    double noise = (-err) + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(err - (-err))));
    return noise;
}

/**
 * @brief getRotationMatrix
 * @param q
 * @return
 */
Eigen::Matrix3d Utils::getRotationMatrix(Eigen::Vector4d q) {
  Eigen::Matrix3d m;
  double qw, qx, qy, qz;
  double qq = q.norm();
  if(qq > 0) {
    qw = q[0] / qq;
    qx = q[1] / qq;
    qy = q[2] / qq;
    qz = q[3] / qq;
  }
  else {
    qw = 1;
    qx = 0;
    qy = 0;
    qz = 0;
  }
  m(0, 0) = qw * qw + qx * qx - qy * qy - qz * qz;
  m(0, 1) = 2 * qx * qy - 2 * qw * qz;
  m(0, 2) = 2 * qw * qy + 2 * qx * qz;
  m(1, 0) = 2 * qx * qy + 2 * qw * qz;
  m(1, 1) = qw * qw + qy * qy - qx * qx - qz * qz;
  m(1, 2) = 2 * qy * qz - 2 * qw * qx;
  m(2, 0) = 2 * qx * qz - 2 * qw * qy;
  m(2, 1) = 2 * qy * qz + 2 * qw * qx;
  m(2, 2) = qw * qw + qz * qz - qx * qx - qy * qy;
  return m;
}

/**
 * @brief getCameraCenterAfterRotation
 * @param c
 * @param r
 * @return
 */
Eigen::Vector3d Utils::getCameraCenterAfterRotation(const double c[3], Eigen::Matrix3d r) {
    Eigen::Vector3d t;
    for(unsigned short int i = 0; i < 3; ++i) {
        t[i] = -(r(i, 0) * c[0] + r(i, 1) * c[1] + r(i, 2) * c[2]);
    }
    return t;
}

double Utils::rad2deg (double alpha) { return (alpha * 57.29578); }    /// from PCL library
double Utils::deg2rad (double alpha) { return (alpha * 0.017453293); } /// from PCL library

/**
 * Computes the normalized value of an angle, which is the equivalent angle in the range ( -Pi, Pi ].
 * @param z	the angle to normalize
 * @return an equivalent angle in the range (-Pi, Pi]
 */
double Utils::normalize_angle(double z)
{
  return atan2(sin(z),cos(z));
}

/**
 * Computes the unoriented smallest difference between two angles.
 *
 * @param a the angle of one vector
 * @param b the angle of the other vector
 * @return the angle (in radians) between the two vectors (in range [0, Pi] )
 */
double Utils::angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize_angle(a);
  b = normalize_angle(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

double Utils::box_muller(double m, double s)	/* normal random variate generator */
{				        /* mean m, standard deviation s */
    double x1, x2, w, y1;
    static double y2;
    static int use_last = 0;

    if (use_last)		        /* use value from previous call */
    {
        y1 = y2;
        use_last = 0;
    }
    else
    {
        do {
            x1 = 2.0 * drand48() - 1.0;
            x2 = 2.0 * drand48() - 1.0;
            w = x1 * x1 + x2 * x2;
        } while ( w >= 1.0 );

        w = sqrt( (-2.0 * log( w ) ) / w );
        y1 = x1 * w;
        y2 = x2 * w;
        use_last = 1;
    }

    return( m + y1 * s );
}

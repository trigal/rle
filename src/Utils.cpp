#include <stdlib.h>
#include <math.h>
#include <Eigen/Eigen>
#include "Utils.h"

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

#ifndef UTILS_H
#define UTILS_H

#include <stdlib.h>
#include <math.h>

double rad2deg (double alpha) { return (alpha * 57.29578); }    /// from PCL library
double deg2rad (double alpha) { return (alpha * 0.017453293); } /// from PCL library

/**
 * Computes the normalized value of an angle, which is the equivalent angle in the range ( -Pi, Pi ].
 * @param z	the angle to normalize
 * @return an equivalent angle in the range (-Pi, Pi]
 */
double normalize_angle(double z)
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
double angle_diff(double a, double b)
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
#endif // UTILS_H

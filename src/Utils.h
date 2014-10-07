#ifndef UTILS_H
#define UTILS_H

#include <stdlib.h>
#include <math.h>
#include <Eigen/Eigen>

class Utils{
public:
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

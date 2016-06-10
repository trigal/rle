#ifndef EDGE_HPP
#define EDGE_HPP

#include "particle/Particle.h"

struct edge
{
    Eigen::Vector3d A, B, centroid;
    Eigen::Vector3d abc;
    double_t d;
    double_t length;

    edge(geometry_msgs::Point _A, geometry_msgs::Point _B, tf::Stamped<tf::Pose> obsPose)
    {
        A = Eigen::Vector3d(_A.x, _A.y, _A.z);
        B = Eigen::Vector3d(_B.x, _B.y, _B.z);

        Eigen::Affine3d TRASL = Eigen::Affine3d::Identity();
        TRASL.translation() << -obsPose.getOrigin().x(), -obsPose.getOrigin().y(), 0;

        A = TRASL * A;
        B = TRASL * B;

        centroid = Eigen::Vector3d(0.5 * A[0] + 0.5 * B[0],
                                   0.5 * A[1] + 0.5 * B[1],
                                   0.5 * A[2] + 0.5 * B[2]);

        Eigen::Affine3d ROT = Eigen::Affine3d::Identity();
        double theta = tf::getYaw(obsPose.getRotation());

        ///Senza il meno sembra avere un senso
        ROT.rotate (Eigen::AngleAxisd (-theta, Eigen::Vector3d::UnitZ()));
        //ROT.rotate (Eigen::AngleAxisd (theta, Eigen::Vector3d::UnitZ()));

        A = ROT * A;
        B = ROT * B;
        centroid = ROT * centroid;

        Eigen::Vector3d z(0.0, 0.0, 1.0);

        Eigen::Vector3d u = B - A;
        length = u.norm(); //Larghezza della facciata
        abc = u.cross(z);
        d = -(abc[0] * A[0] + abc[1] * A[1] + abc[2] * A[2]);
    }
};


#endif // EDGE_HPP

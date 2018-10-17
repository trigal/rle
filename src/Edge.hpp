#ifndef EDGE_HPP
#define EDGE_HPP

#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <tf/tf.h>

#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

using boost::lexical_cast;
using boost::uuids::random_generator;
using boost::uuids::uuid;


struct edge
{
    Eigen::Vector3d A, B, centroid;
    Eigen::Vector3d abc;
    double_t d;
    double_t length;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    double cloud_density = 0.25; //0.5
    double cloud_height = 1.8;

    edge(geometry_msgs::Point _A, geometry_msgs::Point _B, tf::Stamped<tf::Pose> obsPose)
    {
        A = Eigen::Vector3d(_A.x, _A.y, _A.z);
        B = Eigen::Vector3d(_B.x, _B.y, _B.z);

        Eigen::Affine3d TRASL = Eigen::Affine3d::Identity();
        TRASL.translation() << -obsPose.getOrigin().x(), -obsPose.getOrigin().y(), 0;

        A = TRASL * A;
        B = TRASL * B;

        Eigen::Affine3d ROT = Eigen::Affine3d::Identity();
        double theta = tf::getYaw(obsPose.getRotation());

        ///Senza il meno sembra avere un senso
        ROT.rotate (Eigen::AngleAxisd (-theta, Eigen::Vector3d::UnitZ()));
        //ROT.rotate (Eigen::AngleAxisd (theta, Eigen::Vector3d::UnitZ()));

        //A = ROT * A;
        //B = ROT * B;

        //double edge_slope = (B[1] - A[1]) / (B[0] - A[0]);
        //double edge_q = A[1] - edge_slope*A[0];

        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

        double distance = sqrt ((A[0] - B[0])*(A[0] - B[0]) + (A[1] - B[1])*(A[1] - B[1]) );
        int number_of_points = floor( distance / cloud_density );
        double x_diff = (A[0] - B[0]) / number_of_points;
        double y_diff = (A[1] - B[1]) / number_of_points;

        pcl::PointXYZ point;
        point.x = A[0];
        point.y = A[1];
        point.z = 0;
        cloud->push_back(point);
        point.x = B[0];
        point.y = B[1];
        point.z = 0;
        cloud->push_back(point);

        std::cout << "matlab;" << A[0] << ";" << A[1] << ";" << B[0] << ";" << B[1] << std::endl;

        for(int ppp = 0; ppp < number_of_points; ppp++)
        {
            point.x = A[0] - x_diff*ppp;
            point.y = A[1] - y_diff*ppp;
            for(double z = -1.8; z<cloud_height;z+=cloud_density)
            {
                point.z = z;
                cloud->push_back(point);
            }
        }

        //     for(double x = A[0]; x < B[0];x+=cloud_density)
        //     {
        //       for(double z = 0; z<cloud_height;z+=cloud_density)
        //       {
        //         pcl::PointXYZ point;
        //         point.x = x;
        //         point.y = edge_slope*x + edge_q;
        //         point.z = z;
        //         cloud->push_back(point);
        //       }
        //     }
    }
};


#endif // EDGE_HPP



//      edge(geometry_msgs::Point _A, geometry_msgs::Point _B, tf::Stamped<tf::Pose> obsPose)
//      {
//          A = Eigen::Vector3d(_A.x, _A.y, _A.z);
//          B = Eigen::Vector3d(_B.x, _B.y, _B.z);
//
//          Eigen::Affine3d TRASL = Eigen::Affine3d::Identity();
//          TRASL.translation() << -obsPose.getOrigin().x(), -obsPose.getOrigin().y(), 0;
//
//          A = TRASL * A;
//          B = TRASL * B;
//
//          centroid = Eigen::Vector3d(0.5 * A[0] + 0.5 * B[0],
//                                     0.5 * A[1] + 0.5 * B[1],
//                                     0.5 * A[2] + 0.5 * B[2]);
//
//          Eigen::Affine3d ROT = Eigen::Affine3d::Identity();
//          double theta = tf::getYaw(obsPose.getRotation());
//
//          ///Senza il meno sembra avere un senso
//          ROT.rotate (Eigen::AngleAxisd (-theta, Eigen::Vector3d::UnitZ()));
//          ROT.rotate (Eigen::AngleAxisd (theta, Eigen::Vector3d::UnitZ()));
//
//          A = ROT * A;
//          B = ROT * B;
//          centroid = ROT * centroid;
//
//          Eigen::Vector3d z(0.0, 0.0, 1.0);
//
//          Eigen::Vector3d u = B - A;
//          length = u.norm(); //Larghezza della facciata
//          abc = u.cross(z);
//          d = -(abc[0] * A[0] + abc[1] * A[1] + abc[2] * A[2]);
//
//          double edge_slope = (B[1] - A[1]) / (B[0] - A[0]);
//          double edge_q = A[1] - edge_slope*A[0];
//
//          cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
//          for(double x = A[0]; x < B[0];x+=cloud_density){
//            for(double z = 0; z<cloud_height;z+=cloud_density){
//              pcl::PointXYZ point;
//              point.x = x;
//              point.y = edge_slope*x + edge_q;
//              point.z = z;
//              cloud->push_back(point);
//            }
//          }
//      }

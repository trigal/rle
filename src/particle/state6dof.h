#ifndef STATE6DOF_H
#define STATE6DOF_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include "../Utils.h"

class State6DOF
{
public:
    State6DOF();
    State6DOF(const nav_msgs::Odometry &odom_msg);

    Eigen::Vector3d _pose;
    Eigen::AngleAxisd _rotation;
    Eigen::Vector3d _translational_velocity;
    Eigen::AngleAxisd _rotational_velocity;

//    State6DOF operator -(State6DOF &predictedMeasure);
    Eigen::MatrixXd subtract(State6DOF &to_be_subtracted);
    State6DOF addVectorXd(Eigen::VectorXd &to_be_added);
    Eigen::VectorXd toVectorXd();
    geometry_msgs::Pose toGeometryMsgPose();
    void addNoise(double position_offset, double orientation_offset, double linear_offset, double angular_offset);
};

#endif // STATE6DOF_H

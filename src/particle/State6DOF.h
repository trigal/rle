#ifndef STATE6DOF_H
#define STATE6DOF_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include "../Utils.h"

using namespace Eigen;

class State6DOF
{

//private:
public:
    Eigen::Vector3d _pose;                      // wrt world
    Eigen::AngleAxisd _rotation;                // wrt world
    Eigen::Vector3d _translational_velocity;    // wrt robot
    Eigen::AngleAxisd _rotational_velocity;     // wrt robot


public:
    // constructor & destructors ------------------------------------------------------
    State6DOF();
    State6DOF(const nav_msgs::Odometry &odom_msg);

    // methods ------------------------------------------------------------------------
    Eigen::MatrixXd subtract_vectXd(State6DOF &to_be_subtracted);
    State6DOF subtract_state6DOF(State6DOF &to_be_subtracted);
    State6DOF add_vectXd(Eigen::VectorXd &to_be_added);
    State6DOF add_state6DOF(State6DOF &to_be_added);
    Eigen::VectorXd toVectorXd();
    geometry_msgs::Pose toGeometryMsgPose();
    void addNoise(double position_offset, double orientation_offset, double linear_offset, double angular_offset);
    void printState(std::string head_string);

    // getters & setters --------------------------------------------------------------
    Vector3d getPose() { return _pose; }
    void setPose(Vector3d pose){ _pose = pose; }

    AngleAxisd getRotation() { return _rotation; }
    void setRotation(AngleAxisd rotation) { _rotation = rotation; }

    Vector3d getTranslationalVelocity() { return _translational_velocity; }
    void setTranslationalVelocity(Vector3d t_velocity) { _translational_velocity = t_velocity; }

    AngleAxisd getRotationalVelocity() { return _rotational_velocity; }
    void setRotationalVelocity(AngleAxisd rot_velocity) { _rotational_velocity = rot_velocity; }


};

#endif // STATE6DOF_H

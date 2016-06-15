#ifndef STATE6DOF_H
#define STATE6DOF_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include "../Utils.h"

using namespace Eigen;

class   State6DOF
{

//private:  //TODO rimettere privato ed usare i getter/setter
public:
    Eigen::Vector3d   _pose;                    // POSE    wrt world (POSITION)
    Eigen::AngleAxisd _rotation;                // POSE    wrt world (ORIENTATION)
    Eigen::Vector3d   _translational_velocity;  // SPEED   wrt robot
    Eigen::AngleAxisd _rotational_velocity;     // SPEED   wrt robot


public:
    // constructor & destructors ------------------------------------------------------
    State6DOF();
    State6DOF(const nav_msgs::Odometry &odom_msg);
    //State6DOF(State6DOF &toCopy)
    //    : _pose(toCopy._pose),
    //      _rotation(toCopy._rotation),
    //      _translational_velocity(toCopy._translational_velocity),
    //      _rotational_velocity(toCopy._rotational_velocity)
    //{
    //
    //}

    // methods ------------------------------------------------------------------------
    Eigen::MatrixXd subtract_vectXd(State6DOF &to_be_subtracted);
    State6DOF subtract_state6DOF(State6DOF &to_be_subtracted);
    State6DOF add_vectXd(Eigen::VectorXd &to_be_added);
    State6DOF add_state6DOF(State6DOF &to_be_added);
    Eigen::VectorXd toVectorXd();
    geometry_msgs::Pose toGeometryMsgPose();

    double getRoll();
    double getPitch();
    double getYaw();

    void addNoise(double position_offset, double orientation_offset, double linear_offset, double angular_offset);
    void printState(std::string head_string);
    void setOrthogonalPoseRotation();   // _rotation
    void setOrthogonalSpeedRotation();  // _rotational_velocity

    // getters & setters --------------------------------------------------------------
    Vector3d getPosition()
    {
        return _pose;
    }
    void setPose(Vector3d pose)
    {
        _pose = pose;
    }

    AngleAxisd getRotation()   //ROS_ASSERT(_rotation.isUnitary()); TODO:RE-ENABLE
    {
        return _rotation;
    }
    void setRotation(AngleAxisd rotation)
    {
        _rotation = rotation;
    }

    Vector3d getTranslationalVelocity()
    {
        return _translational_velocity;
    }
    void setTranslationalVelocity(Vector3d t_velocity)
    {
        _translational_velocity = t_velocity;
    }

    AngleAxisd getRotationalVelocity()   //ROS_ASSERT(_rotational_velocity.isUnitary()); TODO:RE-ENABLE
    {
        return _rotational_velocity;
    }
    void setRotationalVelocity(AngleAxisd rot_velocity)
    {
        _rotational_velocity = rot_velocity;
    }


};

#endif // STATE6DOF_H

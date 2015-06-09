/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Author:    Dario Limongi                                              *
 *   Email:     dario.limongi@gmail.com                                    *
 *   Date:      02/06/2014                                                 *
 *                                                                         *
 ***************************************************************************/

#include "../Utils.h"
#include "MotionModel.h"
#include "LayoutComponent.h"
#include "../LayoutManager.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
using namespace std;
using namespace Eigen;

const double PI = 3.14159265358979323846;
const double PI_TIMES_2 = 2.0 * PI;
/**
 * Computes the normalized value of an angle, which is the equivalent angle in the range ( -Pi, Pi ].
 * @param angle	the angle to normalize
 * @return an equivalent angle in the range (-Pi, Pi]
 */
double normalize(double angle)
{
    while (angle > PI)
        angle -= PI_TIMES_2;
    while (angle <= -PI)
        angle += PI_TIMES_2;
    return angle;
}

/**
 * @brief this function is used by the particle filter in order to propagate components poses
 * @param p_component
 */

double MotionModel::getPropagate_translational_percentage_vel_error_x() const
{
    return propagate_translational_percentage_vel_error_x;
}

void MotionModel::setPropagate_translational_percentage_vel_error_x(double value)
{
    propagate_translational_percentage_vel_error_x = value;
}

double MotionModel::getPropagate_translational_percentage_vel_error_y() const
{
    return propagate_translational_percentage_vel_error_y;
}

void MotionModel::setPropagate_translational_percentage_vel_error_y(double value)
{
    propagate_translational_percentage_vel_error_y = value;
}

double MotionModel::getPropagate_translational_percentage_vel_error_z() const
{
    return propagate_translational_percentage_vel_error_z;
}

void MotionModel::setPropagate_translational_percentage_vel_error_z(double value)
{
    propagate_translational_percentage_vel_error_z = value;
}

double MotionModel::getPropagate_rotational_percentage_vel_error() const
{
    return propagate_rotational_percentage_vel_error;
}

void MotionModel::setPropagate_rotational_percentage_vel_error(double value)
{
    propagate_rotational_percentage_vel_error = value;
}
VectorXd MotionModel::propagateComponent(VectorXd& pc_state){

    //    VectorXd new_pose = this->propagatePose(pc_state);
    //    std::cout << " ******* PROPAGATED COMPONENT *******" << std::endl;\
    //    std::cout << " Position:" << std::endl;
    //    std::cout << "  x: " << pose(0) << std::endl;
//    std::cout << "  y: " << pose(1) << std::endl;
//    std::cout << "  z: " << pose(2) << std::endl;
//    std::cout << " Orientation quaternion: " << std::endl;
//    std::cout << "  roll: " << pose(3) << std::endl;
//    std::cout << "  pitch: " << pose(4) << std::endl;
//    std::cout << "  yaw: " << pose(5) << std::endl;
//    std::cout << "  z: " << pose(6) << std::endl;
//    std::cout << " Linear speed: " << std::endl;
//    std::cout << "  x: " << pose(6) << std::endl;
//    std::cout << "  y: " << pose(7) << std::endl;
//    std::cout << "  z: " << pose(8) << std::endl;
//    std::cout << " Angular speed: " << std::endl;
//    std::cout << "  x: " << pose(9) << std::endl;
//    std::cout << "  y: " << pose(10) << std::endl;
//    std::cout << "  z: " << pose(11) << std::endl;
//    std::cout << std::endl;

//    return new_pose;
    return pc_state;
}


State6DOF MotionModel::propagatePoseWithControlPercentageAndDelta(State6DOF& p_state,State6DOF& control, double deltaTime)
{
    // MOTION EQUATION:
    // s_t+1 = s_t + v_t * %v_t*  dt
    // v_t+1 = v_t + R

    ROS_ASSERT(p_state.getRotation().isUnitary());
    ROS_ASSERT(p_state.getRotationalVelocity().isUnitary());

    ROS_DEBUG_STREAM("State6DOF MotionModel::propagatePoseWithControlPercentage");
    ROS_DEBUG_STREAM("LIBVISO Speeds (xyz,rpy): "   <<  control.getTranslationalVelocity()[0]  << ";" <<
                                                        control.getTranslationalVelocity()[1]  << ";" <<
                                                        control.getTranslationalVelocity()[2]  << ";" <<
                                                        control.getTranslationalVelocity()[3]  << ";" <<
                                                        control.getTranslationalVelocity()[4]  << ";" <<
                                                        control.getTranslationalVelocity()[5]  << ";" <<
                                                        "; dt: " << deltaTime);
    ROS_ASSERT(p_state.getRotation().isUnitary());
    ROS_ASSERT(p_state.getRotationalVelocity().isUnitary());

    // initialize values
    State6DOF p_state_propagated;

    Eigen::Vector3d percentageVelError;
    percentageVelError(0) = Utils::box_muller(1,propagate_translational_percentage_vel_error_x);
    percentageVelError(1) = Utils::box_muller(1,propagate_translational_percentage_vel_error_y);
    percentageVelError(2) = Utils::box_muller(1,propagate_translational_percentage_vel_error_z);

    ROS_DEBUG_STREAM("St.dev for box_muller function:\t\t\t\t" << propagate_translational_percentage_vel_error_x << "\t\t" << propagate_translational_percentage_vel_error_y << "\t\t" << propagate_translational_percentage_vel_error_z);
    ROS_DEBUG_STREAM("Errors (%):\t" << percentageVelError(0) << "\t" << percentageVelError(1) << "\t" << percentageVelError(2));
    ROS_DEBUG_STREAM("State speeds:\t" << p_state._translational_velocity(0) << "\t" << p_state._translational_velocity(1) << "\t" << p_state._translational_velocity(2));

    // propagate _pose
    Eigen::Vector3d speedToApply;
    speedToApply(0) = control._translational_velocity(0) * percentageVelError(0);
    speedToApply(1) = control._translational_velocity(1) * percentageVelError(1);
    speedToApply(2) = control._translational_velocity(2) * percentageVelError(2);
    p_state_propagated._pose = p_state._pose + p_state._rotation * (speedToApply * deltaTime);

    // propagate pose _rotation
    Eigen::AngleAxisd tmp_angle_axis(control.getRotationalVelocity());
    ROS_DEBUG_STREAM("Angle (deg): " << p_state.getRotationalVelocity().angle() << "\tVector: " << p_state.getRotationalVelocity().axis()(0) <<"\t"<< p_state.getRotationalVelocity().axis()(1) << "\t"<< p_state.getRotationalVelocity().axis()(2) );
    tmp_angle_axis.angle() = tmp_angle_axis.angle() * Utils::box_muller(1,propagate_rotational_percentage_vel_error) * deltaTime;
    p_state_propagated._rotation = tmp_angle_axis * control.getRotation();
    ROS_ASSERT(p_state_propagated.getRotation().isUnitary());

    // propagate speed, translational
    p_state_propagated._translational_velocity(0) = p_state._translational_velocity(0) * percentageVelError(0);
    p_state_propagated._translational_velocity(1) = p_state._translational_velocity(1) * percentageVelError(1);
    p_state_propagated._translational_velocity(2) = p_state._translational_velocity(2) * percentageVelError(2);
    ROS_DEBUG_STREAM("Resulting Speeds:\t" << p_state_propagated._translational_velocity(0) << "\t" << p_state_propagated._translational_velocity(1) << "\t" << p_state_propagated._translational_velocity(2));

    // propagate speed, velocity
    p_state_propagated._rotational_velocity.angle()    = control.getRotationalVelocity().angle() * Utils::box_muller(1,propagate_rotational_percentage_vel_error);
    p_state_propagated._rotational_velocity.axis()(0) *= Utils::box_muller(1,propagate_rotational_percentage_vel_error);
    p_state_propagated._rotational_velocity.axis()(1) *= Utils::box_muller(1,propagate_rotational_percentage_vel_error);
    p_state_propagated._rotational_velocity.axis().normalize();
    ROS_ASSERT(p_state_propagated.getRotationalVelocity().isUnitary());


    return p_state_propagated;
}

State6DOF MotionModel::propagatePoseWithControl(State6DOF& p_state,State6DOF& control)
{
    // MOTION EQUATION:
    // s_t+1 = s_t + v_t * Delta_t + R
    // v_t+1 = v_t + R

    ROS_ASSERT(p_state.getRotation().isUnitary());
    ROS_ASSERT(p_state.getRotationalVelocity().isUnitary());

    ROS_DEBUG_STREAM("State6DOF MotionModel::propagatePoseWithControl");
    ROS_DEBUG_STREAM("LIBVISO Speeds (xyz,rpy): "   <<  control.getTranslationalVelocity()[0]  << ";" <<
                                                        control.getTranslationalVelocity()[1]  << ";" <<
                                                        control.getTranslationalVelocity()[2]  << ";" <<
                                                        control.getTranslationalVelocity()[3]  << ";" <<
                                                        control.getTranslationalVelocity()[4]  << ";" <<
                                                        control.getTranslationalVelocity()[5]  << ";" <<
                                                        "; dt: " << LayoutManager::deltaOdomTime);

    // initialize values
    State6DOF p_state_propagated;

    Eigen::Vector3d absolute_error; //Absolute Error
    absolute_error(0) = Utils::box_muller(0,propagate_translational_vel_error_x);
    absolute_error(1) = Utils::box_muller(0,propagate_translational_vel_error_y);
    absolute_error(2) = Utils::box_muller(0,propagate_translational_vel_error_z);

    ROS_DEBUG_STREAM("St.dev for box_muller function:\t\t\t\t" << propagate_translational_vel_error_x << "\t\t" << propagate_translational_vel_error_y << "\t\t" << propagate_translational_vel_error_z);
    ROS_DEBUG_STREAM("Translational sampled errors:\t\t\t\t" << absolute_error(0) << "\t" << absolute_error(1) << "\t" << absolute_error(2));

    // propagate _pose
    if(control.getTranslationalVelocity()[0] < 0) //we have X-FORWARD messages
    {
        ROS_WARN_STREAM("LIBVISO failure: " << control.getTranslationalVelocity()[0] << " == " << LayoutManager::deltaOdomTime);

        p_state_propagated._pose = p_state._pose + p_state._rotation * (p_state._translational_velocity * LayoutManager::deltaOdomTime) + absolute_error;

        // propagate pose _rotation
        Eigen::AngleAxisd tmp_angle_axis(p_state._rotational_velocity);
        tmp_angle_axis.angle() = tmp_angle_axis.angle() * LayoutManager::deltaOdomTime + Utils::box_muller(0,propagate_rotational_vel_error);
        p_state_propagated._rotation = tmp_angle_axis * p_state._rotation;

        // Generate random error with box_muller function

        // propagate velocity
        p_state_propagated._translational_velocity = p_state._translational_velocity; // WARNING + verify error;

        p_state_propagated._rotational_velocity = p_state._rotational_velocity;
        p_state_propagated._rotational_velocity.angle() = p_state_propagated._rotational_velocity.angle();// + Utils::box_muller(0,propagate_rotational_vel_error);
    }
    else
    {
        // propagate pose (translation part)
        p_state_propagated._pose = p_state._pose + p_state._rotation * (control._translational_velocity * LayoutManager::deltaOdomTime) + absolute_error;

        // propagate pose (rotation part)
        Eigen::AngleAxisd tmp_angle_axis(control._rotational_velocity);
        tmp_angle_axis.angle() = tmp_angle_axis.angle() * LayoutManager::deltaOdomTime + Utils::box_muller(0,propagate_rotational_vel_error);
        p_state_propagated._rotation = tmp_angle_axis * p_state._rotation;

        // Generate random error with box_muller function

        // propagate velocity
        p_state_propagated._translational_velocity = control._translational_velocity;// + tmp_error; // WARNING + verify error;

        p_state_propagated._rotational_velocity = control._rotational_velocity;
        p_state_propagated._rotational_velocity.angle() = p_state_propagated._rotational_velocity.angle();// + Utils::box_muller(0,propagate_rotational_vel_error);
    }

    return p_state_propagated;
}

/**
 * @brief this function is used by EKF in order to propagate particle's pose
 * @param p_state
 * @return p_state_predicted
 */
State6DOF MotionModel::propagatePoseWithPercentage(State6DOF& p_state)
{

    ROS_ASSERT(p_state.getRotation().isUnitary());
    ROS_ASSERT(p_state.getRotationalVelocity().isUnitary());

    // MOTION EQUATION:
    // s_t+1 = s_t + v_t * Delta_t + R
    // v_t+1 = v_t + R

    // initialize values
    State6DOF p_state_propagated;

    Eigen::Vector3d percentageVelError;
    percentageVelError(0) = Utils::box_muller(1,propagate_translational_percentage_vel_error_x);
    percentageVelError(1) = Utils::box_muller(1,propagate_translational_percentage_vel_error_y);
    percentageVelError(2) = Utils::box_muller(1,propagate_translational_percentage_vel_error_z);

    ROS_DEBUG_STREAM("St.dev for box_muller function:\t\t\t\t" << propagate_translational_percentage_vel_error_x << "\t\t" << propagate_translational_percentage_vel_error_y << "\t\t" << propagate_translational_percentage_vel_error_z);
    ROS_DEBUG_STREAM("Errors:\t" << percentageVelError(0) << "\t" << percentageVelError(1) << "\t" << percentageVelError(2));
    ROS_DEBUG_STREAM("State speeds:\t" << p_state._translational_velocity(0) << "\t" << p_state._translational_velocity(1) << "\t" << p_state._translational_velocity(2));

    // propagate _pose
    p_state_propagated._pose = p_state._pose + p_state._rotation * (p_state._translational_velocity * LayoutManager::deltaOdomTime);
    p_state_propagated._pose(0)+=Utils::box_muller(0,propagate_translational_vel_error_x);
    p_state_propagated._pose(1)+=Utils::box_muller(0,propagate_translational_vel_error_y);
    p_state_propagated._pose(2)+=Utils::box_muller(0,propagate_translational_vel_error_z);

    // propagate pose _rotation
    Eigen::AngleAxisd tmp_angle_axis(p_state._rotational_velocity);
    ROS_DEBUG_STREAM("Angle (deg): " << p_state.getRotationalVelocity().angle() << "\tVector: " << p_state.getRotationalVelocity().axis()(0) <<"\t"<< p_state.getRotationalVelocity().axis()(1) << "\t"<< p_state.getRotationalVelocity().axis()(2) );
    tmp_angle_axis.angle() = tmp_angle_axis.angle() * Utils::box_muller(1,propagate_rotational_percentage_vel_error) * LayoutManager::deltaOdomTime;
    p_state_propagated._rotation = tmp_angle_axis * p_state._rotation ;
    ROS_ASSERT(p_state_propagated.getRotation().isUnitary());

    // Generate random error with box_muller function
    p_state_propagated._translational_velocity(0) = p_state._translational_velocity(0) * percentageVelError(0);
    p_state_propagated._translational_velocity(1) = p_state._translational_velocity(1) * percentageVelError(1);
    p_state_propagated._translational_velocity(2) = p_state._translational_velocity(2) * percentageVelError(2);
    ROS_DEBUG_STREAM("Resulting Speeds:\t" << p_state._translational_velocity(0) << "\t" << p_state._translational_velocity(1) << "\t" << p_state._translational_velocity(2));

    p_state_propagated._rotational_velocity = p_state._rotational_velocity;
    p_state_propagated._rotational_velocity.angle() = p_state_propagated._rotational_velocity.angle() * Utils::box_muller(1,propagate_rotational_percentage_vel_error);
    p_state_propagated._rotational_velocity.axis()(0) *= Utils::box_muller(1,propagate_rotational_percentage_vel_error);
    p_state_propagated._rotational_velocity.axis()(1) *= Utils::box_muller(1,propagate_rotational_percentage_vel_error);
    //p_state_propagated._rotational_velocity.axis()(2) *= Utils::box_muller(1,propagate_rotational_percentage_vel_error);
    p_state_propagated._rotational_velocity.axis().normalize();
    ROS_ASSERT(p_state_propagated.getRotationalVelocity().isUnitary());

    return p_state_propagated;
}

State6DOF MotionModel::propagatePoseWithPercentageAndDeltatime(State6DOF& p_state, double deltaTime)
{
    ROS_DEBUG_STREAM("> entering propagatePoseWithPercentageAndDelta");
    ROS_ASSERT(p_state.getRotation().isUnitary());
    ROS_ASSERT(p_state.getRotationalVelocity().isUnitary());

    // MOTION EQUATION:
    // s_t+1 = s_t + v_t * Delta_t + R
    // v_t+1 = v_t + R

    // initialize values
    State6DOF p_state_propagated;
    Eigen::Vector3d percentageVelError;
    percentageVelError(0) = Utils::box_muller(1,propagate_translational_percentage_vel_error_x);
    percentageVelError(1) = Utils::box_muller(1,propagate_translational_percentage_vel_error_y);
    percentageVelError(2) = Utils::box_muller(1,propagate_translational_percentage_vel_error_z);

    ROS_DEBUG_STREAM("St.dev for box_muller function:\t\t\t\t" << propagate_translational_percentage_vel_error_x << "\t\t" << propagate_translational_percentage_vel_error_y << "\t\t" << propagate_translational_percentage_vel_error_z);
    ROS_DEBUG_STREAM("Errors:\t" << percentageVelError(0) << "\t" << percentageVelError(1) << "\t" << percentageVelError(2));
    ROS_DEBUG_STREAM("State speeds:\t" << p_state._translational_velocity(0) << "\t" << p_state._translational_velocity(1) << "\t" << p_state._translational_velocity(2));

    // propagate _pose
    p_state_propagated._pose = p_state._pose + p_state._rotation * (p_state._translational_velocity * deltaTime);
    p_state_propagated._pose(0)+=Utils::box_muller(0,propagate_translational_vel_error_x);
    p_state_propagated._pose(1)+=Utils::box_muller(0,propagate_translational_vel_error_y);
    p_state_propagated._pose(2)+=Utils::box_muller(0,propagate_translational_vel_error_z);
    BUG qui sommo xyz su sistema di cordinate ad cazzium.
    // propagate _pose
    //Eigen::Vector3d speedToApply;
    //speedToApply(0) = p_state._translational_velocity(0) * percentageVelError(0);
    //speedToApply(1) = p_state._translational_velocity(1) * percentageVelError(1);
    //speedToApply(2) = p_state._translational_velocity(2) * percentageVelError(2);
    //p_state_propagated._pose = p_state._pose + p_state._rotation * (speedToApply * deltaTime);
    //p_state_propagated._pose(0)+=Utils::box_muller(0,propagate_translational_vel_error_x); ///adding error
    //p_state_propagated._pose(1)+=Utils::box_muller(0,propagate_translational_vel_error_y); ///adding error
    //p_state_propagated._pose(2)+=Utils::box_muller(0,propagate_translational_vel_error_z); ///adding error

    // propagate pose _rotation
    Eigen::AngleAxisd tmpAngleAxisd(p_state._rotational_velocity);
    ROS_DEBUG_STREAM("Angle (deg): " << p_state.getRotationalVelocity().angle() << "\tVector: " << p_state.getRotationalVelocity().axis()(0) <<"\t"<< p_state.getRotationalVelocity().axis()(1) << "\t"<< p_state.getRotationalVelocity().axis()(2) );
    tmpAngleAxisd.angle() = tmpAngleAxisd.angle() * Utils::box_muller(1,propagate_rotational_percentage_vel_error) * deltaTime;
    tmpAngleAxisd.angle() += Utils::box_muller(0,propagate_rotational_vel_error);         ///adding error
    anche quick_exit();
    //tmpAngleAxisd.axis()(0) *= Utils::box_muller(1,propagate_rotational_percentage_vel_error)*0.01;
    //tmpAngleAxisd.axis()(1) *= Utils::box_muller(1,propagate_rotational_percentage_vel_error)*0.01;
    //tmpAngleAxisd.axis().normalize();
    p_state_propagated._rotation = tmpAngleAxisd * p_state._rotation ;
    ROS_ASSERT(p_state_propagated.getRotation().isUnitary());

    // Generate random error with box_muller function
    p_state_propagated._translational_velocity(0) = p_state._translational_velocity(0) * percentageVelError(0);
    p_state_propagated._translational_velocity(1) = p_state._translational_velocity(1) * percentageVelError(1);
    p_state_propagated._translational_velocity(2) = p_state._translational_velocity(2) * percentageVelError(2);
    ROS_DEBUG_STREAM("Resulting Speeds:\t" << p_state_propagated._translational_velocity(0) << "\t" << p_state_propagated._translational_velocity(1) << "\t" << p_state_propagated._translational_velocity(2));

    p_state_propagated._rotational_velocity = p_state._rotational_velocity;
    p_state_propagated._rotational_velocity.angle() = p_state_propagated._rotational_velocity.angle() * Utils::box_muller(1,propagate_rotational_percentage_vel_error);
    //p_state_propagated._rotational_velocity.axis()(0) *= Utils::box_muller(1,propagate_rotational_percentage_vel_error)*0.01;
    //p_state_propagated._rotational_velocity.axis()(1) *= Utils::box_muller(1,propagate_rotational_percentage_vel_error)*0.01;
    //p_state_propagated._rotational_velocity.axis()(2) *= Utils::box_muller(1,propagate_rotational_percentage_vel_error);
    //p_state_propagated._rotational_velocity.axis().normalize();
    ROS_ASSERT(p_state_propagated.getRotationalVelocity().isUnitary());

    ROS_DEBUG_STREAM("< exiting propagatePoseWithPercentageAndDelta");
    return p_state_propagated;
}


/**
 * @brief this function is used by EKF in order to propagate particle's pose
 * @param p_state
 * @return p_state_predicted
 */
State6DOF MotionModel::propagatePoseWithAbsolute(State6DOF& p_state)
{

    ROS_ASSERT(p_state.getRotation().isUnitary());
    ROS_ASSERT(p_state.getRotationalVelocity().isUnitary());

    // MOTION EQUATION:
    // s_t+1 = s_t + v_t * Delta_t + R
    // v_t+1 = v_t + R

    // initialize values
    State6DOF p_state_propagated;

    // propagate _pose
    p_state_propagated._pose = p_state._pose + p_state._rotation * (p_state._translational_velocity * LayoutManager::deltaOdomTime);

    // propagate pose _rotation
    Eigen::AngleAxisd tmp_angle_axis(p_state.getRotationalVelocity());
    tmp_angle_axis.angle() = tmp_angle_axis.angle() * LayoutManager::deltaOdomTime;
    p_state_propagated._rotation = tmp_angle_axis * p_state._rotation;

    // Generate random error with box_muller function
    Eigen::Vector3d tmp_error;
    tmp_error(0) = Utils::box_muller(0,propagate_translational_vel_error_x);
    tmp_error(1) = Utils::box_muller(0,propagate_translational_vel_error_y);
    tmp_error(2) = Utils::box_muller(0,propagate_translational_vel_error_z);

    // propagate velocity
    p_state_propagated._translational_velocity = p_state._translational_velocity + tmp_error; // WARNING + verify error;

    p_state_propagated._rotational_velocity = p_state._rotational_velocity;
    p_state_propagated._rotational_velocity.angle() = p_state_propagated._rotational_velocity.angle() + Utils::box_muller(0,propagate_rotational_vel_error);

    ROS_ASSERT(p_state_propagated.getRotation().isUnitary());
    ROS_ASSERT(p_state_propagated.getRotationalVelocity()isUnitary());

    return p_state_propagated;
}


MatrixXd MotionModel::motionJacobian(State6DOF& p_state_predicted){
    /**
     * G_t:
     *
     *  | 1 0 0 0 0 0  Dt 0 0 0 0 0 |
     *  | 0 1 0 0 0 0  0 Dt 0 0 0 0 |
     *  | 0 0 1 0 0 0  0 0 Dt 0 0 0 |
     *  | 0 0 0 1 0 0  0 0 0 Dt 0 0 |
     *  | 0 0 0 0 1 0  0 0 0 0 Dt 0 |
     *  | 0 0 0 0 0 1  0 0 0 0 0 Dt |
     *
     * 	| 0 0 0 0 0 0  1 0 0 0 0 0 |
     *  | 0 0 0 0 0 0  0 1 0 0 0 0 |
     *  | 0 0 0 0 0 0  0 0 1 0 0 0 |
     *  | 0 0 0 0 0 0  0 0 0 1 0 0 |
     *  | 0 0 0 0 0 0  0 0 0 0 1 0 |
     *  | 0 0 0 0 0 0  0 0 0 0 0 1 |
     */


    // Create an identity 12x12 matrix
    MatrixXd G_t = MatrixXd::Identity(12,12);

    // Sets diagonal of first 6x6 matrix to delta_t
    for(int i = 0; i<6; i++)
        G_t(i,i+6) = LayoutManager::deltaOdomTime;

    return G_t;
}

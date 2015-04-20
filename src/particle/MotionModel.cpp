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


/**
 * @brief this function is used by EKF in order to propagate particle's pose
 * @param p_state
 * @return p_state_predicted
 */
State6DOF MotionModel::propagatePose(State6DOF& p_state){
    // MOTION EQUATION:
    // s_t+1 = s_t + v_t * Delta_t + R
    // v_t+1 = v_t + R

    // initialize values
    State6DOF p_state_propagated;

    // propagate _pose
    p_state_propagated._pose = p_state._pose + p_state._rotation * (p_state._translational_velocity * LayoutManager::delta_t);

    // propagate pose _rotation
    Eigen::AngleAxisd tmp_angle_axis(p_state._rotational_velocity);
    tmp_angle_axis.angle() = tmp_angle_axis.angle() * LayoutManager::delta_t;
    p_state_propagated._rotation = tmp_angle_axis * p_state._rotation;

    // Generate random error with box_muller function
    p_state_propagated._translational_velocity(0) = p_state._translational_velocity(0) * Utils::box_muller(1,propagate_translational_percentage_vel_error_x);
    p_state_propagated._translational_velocity(1) = p_state._translational_velocity(1) * Utils::box_muller(1,propagate_translational_percentage_vel_error_y);
    p_state_propagated._translational_velocity(2) = p_state._translational_velocity(2) * Utils::box_muller(1,propagate_translational_percentage_vel_error_z);


    p_state_propagated._rotational_velocity = p_state._rotational_velocity;
    p_state_propagated._rotational_velocity.angle() = p_state_propagated._rotational_velocity.angle() * Utils::box_muller(1,propagate_rotational_percentage_vel_error);
    p_state_propagated._rotational_velocity.axis()(0) *= Utils::box_muller(1,propagate_rotational_percentage_vel_error);
    p_state_propagated._rotational_velocity.axis()(1) *= Utils::box_muller(1,propagate_rotational_percentage_vel_error);
    p_state_propagated._rotational_velocity.axis()(2) *= Utils::box_muller(1,propagate_rotational_percentage_vel_error);
    p_state_propagated._rotational_velocity.axis().normalize();

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
        G_t(i,i+6) = LayoutManager::delta_t;

    return G_t;
}

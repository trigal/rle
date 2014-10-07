/*
 * MotionModel.cpp
 *
 *  Created on: Jun 2, 2014
 *      Author: dario
 */

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

    VectorXd new_pose = this->propagatePose(pc_state);
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

    return new_pose;
}


/**
 * @brief this function is used by EKF in order to propagate particle's pose
 * @param p_state
 * @return p_state_predicted
 */
VectorXd MotionModel::propagatePose(VectorXd& p_state){
    // MOTION EQUATION:
	// s_t+1 = s_t + v_t * Delta_t + R
    // v_t+1 = v_t + R

	// initialize values
	VectorXd p_pose = VectorXd::Zero(6);
	VectorXd p_vel = VectorXd::Zero(6);
	VectorXd pose_error = VectorXd::Zero(6);
	VectorXd vel_error = VectorXd::Zero(6);
	VectorXd p_state_propagated = VectorXd::Zero(12);

	// build values from p_state
	p_pose = p_state.head(6);
	p_vel = p_state.tail(6);
	pose_error = error_covariance.diagonal().head(6);
	vel_error = error_covariance.diagonal().tail(6);

    // get a random error value from error_covariance matrix
    for(int i=0; i<pose_error.size(); i++)
    {
        // temp variables
        double p_err = pose_error(i);
        double v_err = vel_error(i);

        // rand values that will be added to calculated pose/vel
        pose_error(i) = Utils::getNoise(p_err);
        vel_error(i) = Utils::getNoise(v_err);
    }

	// propagate p_pose
    p_pose = p_pose + (p_vel * LayoutManager::delta_t);// + pose_error; //random
    p_vel = p_vel;// + vel_error;

    // normalize angles -PI, PI
    p_pose(3) = normalize(p_pose(3));
    p_pose(4) = normalize(p_pose(4));
    p_pose(5) = normalize(p_pose(5));

	// build propagated p_state
	p_state_propagated << p_pose, p_vel;

	return p_state_propagated;
}


MatrixXd MotionModel::motionJacobi(VectorXd& p_state_predicted){
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

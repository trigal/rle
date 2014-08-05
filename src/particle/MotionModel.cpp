/*
 * MotionModel.cpp
 *
 *  Created on: Jun 2, 2014
 *      Author: dario
 */

#include "MotionModel.h"
#include "ParticleComponent.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
using namespace std;
using namespace Eigen;

void MotionModel::propagateComponent(ParticleComponent* p_component){
	//motion-model
	cout << "Propagating component" << endl;
}

VectorXd MotionModel::propagatePose(VectorXd& p_state){
	// X_t ----> X_t+1
	//
	// s_t+1 = s_t + v_t * Delta_t + R

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

	// propagate p_pose
    p_pose = p_pose + (p_vel * delta_t) + pose_error;
	p_vel = p_vel + vel_error;

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
		G_t(i,i+6) = delta_t;

	return G_t;
}


MatrixXd MotionModel::measurementJacobi(VectorXd& p_state_predicted){
	/**
	 * H_t:
	 *
	 *  | 1 0 0 0 0 0  0 0 0 0 0 0 |
	 *  | 0 1 0 0 0 0  0 0 0 0 0 0 |
	 *  | 0 0 1 0 0 0  0 0 0 0 0 0 |
	 *  | 0 0 0 1 0 0  0 0 0 0 0 0 |
	 *  | 0 0 0 0 1 0  0 0 0 0 0 0 |
	 *  | 0 0 0 0 0 1  0 0 0 0 0 0 |
	 *
	 * 	| 0 0 0 0 0 0  1 0 0 0 0 0 |
	 *  | 0 0 0 0 0 0  0 1 0 0 0 0 |
	 *  | 0 0 0 0 0 0  0 0 1 0 0 0 |
	 *  | 0 0 0 0 0 0  0 0 0 1 0 0 |
	 *  | 0 0 0 0 0 0  0 0 0 0 1 0 |
	 *  | 0 0 0 0 0 0  0 0 0 0 0 1 |
	 */

	// Create an identity 12x12 matrix
	MatrixXd H_t = MatrixXd::Identity(12,12);

	return H_t;
}

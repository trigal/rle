/*
 * MotionModel.h
 *
 *  Created on: Jun 2, 2014
 *      Author: dario
 */

#ifndef MOTIONMODEL_H_
#define MOTIONMODEL_H_

#include "LayoutComponent.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>

using namespace Eigen;
using namespace std;

/**
 * This class is used by every particle in order to propagate itself using the
 * defined motion-model.
 * In case there aren't enough datas from Odometry sensors, a decadency motion model
 * will be applied on particle propagation
 */
class MotionModel {
private:
	MatrixXd error_covariance; 	/// 12x12 (double) error covariance matrix
								/// 6DoF + 6 derivate(velocita su ogni DoF)



	/**
	 * In case there aren't enough datas from Odometry sensors, a decadency motion model
	 * will be applied on particle propagation
	 */
	void decadencyMotionModel();

public:

	/**
	 * This function will propagate the particle component using the defined
	 * motion-model inside MotionModel.cpp
	 * @param p_component
	 */
    VectorXd propagateComponent(VectorXd& pc_state);

    /**
     * @brief propagatePose
     * @param particle_state
     * @return
     */
    VectorXd propagatePose(VectorXd& particle_state);

    /**
     * @brief motionJacobi
     * @param p_state_predicted
     * @return
     */
    MatrixXd motionJacobi(VectorXd& p_state_predicted);

    // getters & setter ------------------------------------------------------------------------
    MatrixXd getErrorCovariance(){ return error_covariance; }
    void setErrorCovariance(MatrixXd& err){ error_covariance = err; }
    void setErrorCovariance(double uncertainty) { error_covariance = MatrixXd::Identity(12,12) * (uncertainty*uncertainty); }

    // constructor & destructor ----------------------------------------------------------------
    MotionModel(const MatrixXd& cov) : error_covariance(cov) {}
    MotionModel(double inc) { error_covariance = MatrixXd::Identity(12,12) * (inc*inc); }
    MotionModel() {
		// Sets the error covariance to zero
		MatrixXd cov = MatrixXd::Zero(12,12);
		error_covariance = cov;
    }

    ~MotionModel(){
        error_covariance.resize(0,0);
    }
};

#endif /* MOTIONMODEL_H_ */

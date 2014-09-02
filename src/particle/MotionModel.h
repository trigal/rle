/*
 * MotionModel.h
 *
 *  Created on: Jun 2, 2014
 *      Author: dario
 */

#ifndef MOTIONMODEL_H_
#define MOTIONMODEL_H_

#include "ParticleComponent.h"
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
    double delta_t;	/// Time interval between t and t+1 (seconds)
	/**
	 * This function will propagate the particle component using the defined
	 * motion-model inside MotionModel.cpp
	 * @param p_component
	 */
	void propagateComponent(ParticleComponent* p_component);

    VectorXd propagatePose(VectorXd& particle_state);

    MatrixXd motionJacobi(VectorXd& p_state_predicted);

	/**
	 * @return current error covariance
	 */
	MatrixXd getErrorCovariance(){
		return error_covariance;
	}

	/**
	 * Sets current error covariance
	 * @param err
	 */
	void setErrorCovariance(MatrixXd& err){
		error_covariance = err;
	}

	/**
	 * Returns on console a string of current motion model values
	 */
	void toString(){
		cout << "Motion model, delta_t = " << delta_t << " error covariance = " << endl;
		cout << error_covariance << endl;
	}

	/**
	 * Istantiate MotionModel with the given error covariance, and delta_t to 30fps if it's not passed as argument
	 * @param cov
	 * @param dt
	 */
    MotionModel(const MatrixXd& cov) : error_covariance(cov), delta_t(0.03333){}
    MotionModel(const MatrixXd& cov, const double dt) : error_covariance(cov), delta_t(dt){}

	/**
	 * Default constructor
	 */
	MotionModel() {
		// Sets delta_t as 30fps -> 33.33 milliseconds = 0.03333 seconds
		delta_t = 0.03333;

		// Sets the error covariance to zero
		MatrixXd cov = MatrixXd::Zero(12,12);
		error_covariance = cov;
    }

	//destructor
	virtual ~MotionModel(){
		//error_covariance.resize(0,0);
    }
};

#endif /* MOTIONMODEL_H_ */

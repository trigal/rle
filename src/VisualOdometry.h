/*
 * VisualOdometry.h
 *
 *  Created on: May 25, 2014
 *      Author: dario
 */

#ifndef VISUALODOMETRY_H_
#define VISUALODOMETRY_H_

#include <Eigen/Dense>	//used for motion threshold matrix
#include <Eigen/Core>
using namespace Eigen;

class VisualOdometry {

private:
	MatrixXd error_covariance; 		/// this is a 12x12 matrix representing the error covariance on the measurement
	VectorXd particle_measurement;	/// particle measurement (12x1: 6DoF pose + 6 Speed Derivates)

	MatrixXd odometry;

	class Tracker{

	};

	class Mapper{

	};

public:

	/**
	 * @param p_state
	 * @return measurement vector of the given particle, obtained using measurement model equations
	 */
	VectorXd measurePose(VectorXd& p_state);

	/**
	 * @return vector 12x1 representing the current measurement of the particle
	 */
	VectorXd getParticleMeasurement(){
		return this->particle_measurement;
	}

	/**
	 * Sets current particle measurement
	 * @param msr
	 */
	void setParticleMeasurement(VectorXd& msr){
		this->particle_measurement = msr;
	}

	/**
	 * @return current error covariance
	 */
	MatrixXd getErrorCovariance(){
		return this->error_covariance;
	}

	/**
	 * Sets current error covariance
	 * @param err
	 */
	void setErrorCovariance(MatrixXd& err){
		this->error_covariance = err;
	}

	/**
	 * @return current odometry
	 */
	MatrixXd getOdometry(){
		return odometry;
	}

	/**
	 * Sets current odometry
	 * @param odm
	 */
	void setOdometry(MatrixXd& odm){
		this->odometry = odm;
	}

	VisualOdometry(){
		// Sets all the values to zero
		MatrixXd cov = MatrixXd::Zero(12,12);
		VectorXd msr = VectorXd::Zero(12);
		MatrixXd odm = MatrixXd::Zero(12,12);

		this->error_covariance = cov;
		this->particle_measurement = msr;
		this->odometry = odm;
	};

	~VisualOdometry(){
		this->odometry.resize(0,0);
	};
};

#endif /* VISUALODOMETRY_H_ */

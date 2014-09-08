/*
 * Odometry.h
 *
 *  Created on: May 25, 2014
 *      Author: dario
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <Eigen/Dense>	//used for motion threshold matrix
#include <Eigen/Core>
using namespace Eigen;

class Odometry {

private:
    MatrixXd error_covariance; 		/// this is a 12x12 matrix representing the error covariance on the measurement
    VectorXd current_measurement;	/// particle measurement (12x1: 6DoF pose + 6 Speed Derivates)

    class Tracker{

    };

    class Mapper{

    };

public:

    /**
     * @brief measurementJacobi
     * @param p_state_predicted
     * @return
     */
    MatrixXd measurementJacobi(VectorXd& p_state_predicted);

    /**
     * @param p_state
     * @return measurement vector of the given particle, obtained using measurement model equations
     */
    VectorXd measurePose(VectorXd& p_state);

    /**
     * @return vector 12x1 representing the current measurement of the particle
     */
    VectorXd getCurrentMeasurement(){
        return this->current_measurement;
    }

    /**
     * Sets current particle measurement
     * @param msr
     */
    void setCurrentMeasurement(VectorXd& msr){
        this->current_measurement = msr;
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

    Odometry(){
        // Sets all the values to zero
        MatrixXd cov = MatrixXd::Zero(12,12);
        VectorXd msr = VectorXd::Zero(12);

        this->error_covariance = cov;
        this->current_measurement = msr;
    }

    ~Odometry(){
        this->current_measurement.resize(0);
        this->error_covariance(0,0);
    }
};

#endif /* ODOMETRY_H_ */

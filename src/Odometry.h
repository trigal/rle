/*
 * Odometry.h
 *
 *  Created on: May 25, 2014
 *      Author: dario
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "Utils.h"

#include <Eigen/Dense>	//used for motion threshold matrix
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>

using namespace Eigen;

class Odometry {

private:
    VectorXd msr_state; /// particle measurement (12x1: 6DoF pose + 6 Speed Derivates)
    MatrixXd msr_cov;   /// particle covariance (12x12)
    nav_msgs::Odometry msg; /// current msg arrived from Visual Odometry

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


    // getters & setters --------------------------------------------------------------------------
    void setMeasureState(VectorXd& msrstate){ msr_state = msrstate; }
    VectorXd getMeasureState(){ return msr_state; }

    void setMeasureCov(MatrixXd& msrcov){ msr_cov = msrcov; }
    void setMeasureCov(double unc){ msr_cov = MatrixXd::Identity(12,12) * (unc*unc); }
    MatrixXd getMeasureCov(){ return msr_cov; }

    void setMsg(const nav_msgs::Odometry& m){
        msg = m;
        msr_state = Utils::getPoseVectorFromOdom(m);
        msr_cov = Utils::getCovFromOdom(m);
        Utils::printOdomMsgToCout(m);
    }
    nav_msgs::Odometry getMsg() { return msg; }

    // constructor & destructor -------------------------------------------------------------------
    Odometry(){
        // Sets all values to zero
        msr_cov = MatrixXd::Zero(12,12);
        msr_state = VectorXd::Zero(12);
    }

    ~Odometry(){
        // Resize all values to zero
        msr_state.resize(0);
        msr_cov.resize(0,0);
    }
};

#endif /* ODOMETRY_H_ */

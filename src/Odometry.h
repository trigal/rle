/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Author:    Dario Limongi                                              *
 *   Email:     dario.limongi@gmail.com                                    *
 *   Date:      25/05/2014                                                 *
 *                                                                         *
 ***************************************************************************/

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "Utils.h"
#include "particle/State6DOF.h"

#include <Eigen/Dense>	//used for motion threshold matrix
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>

using namespace Eigen;

class Odometry {


public:
    State6DOF measurement_state;

private:
//    VectorXd msr_state; /// measurement (12x1: 6DoF pose + 6 Speed Derivates)
    MatrixXd msr_cov;   /// measurment covariance (12x12)
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
    MatrixXd measurementJacobian(State6DOF &p_state_predicted);

    /**
     * @param p_state
     * @return measurement vector of the given particle, obtained using measurement model equations
     */
    State6DOF measurePose(State6DOF &p_state);


    // getters & setters --------------------------------------------------------------------------
    void setMeasureState(State6DOF& mrs){
        measurement_state._pose = mrs._pose;
        measurement_state._rotation = mrs._rotation;
        measurement_state._translational_velocity= mrs._translational_velocity;
        measurement_state._rotational_velocity= mrs._rotational_velocity;
    }

    State6DOF getMeasureState() { return measurement_state; }

    void setMeasureCov(MatrixXd& msrcov){ msr_cov = msrcov; }
    void setMeasureCov(double unc){ msr_cov = MatrixXd::Identity(12,12) * (unc*unc); }
    void setMeasureCov(double pos_unc, double ori_unc, double lin_unc, double ang_unc) {
        msr_cov = MatrixXd::Zero(12,12);
        msr_cov(0,0) = pos_unc*pos_unc;
        msr_cov(1,1) = pos_unc*pos_unc;
        msr_cov(2,2) = pos_unc*pos_unc;
        msr_cov(3,3) = ori_unc*ori_unc;
        msr_cov(4,4) = ori_unc*ori_unc;
        msr_cov(5,5) = ori_unc*ori_unc;
        msr_cov(6,6) = lin_unc*lin_unc;
        msr_cov(7,7) = lin_unc*lin_unc;
        msr_cov(8,8) = lin_unc*lin_unc;
        msr_cov(9,9) = ang_unc*ang_unc;
        msr_cov(10,10) = ang_unc*ang_unc;
        msr_cov(11,11) = ang_unc*ang_unc;
    }
    MatrixXd getMeasureCov(){ return msr_cov; }

    void setMsg(const nav_msgs::Odometry& m){
        msg = m;
        measurement_state = State6DOF(msg);

        msr_cov = Utils::getCovFromOdom(m);
//        Utils::printOdomMsgToCout(m);
    }
    nav_msgs::Odometry getMsg() { return msg; }

    // constructor & destructor -------------------------------------------------------------------
    Odometry(){
        // Sets all values to zero
        msr_cov = MatrixXd::Zero(12,12);
    }

    ~Odometry(){
        // Resize all values to zero
        msr_cov.resize(0,0);
    }
};

#endif /* ODOMETRY_H_ */

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
#include <tf/transform_listener.h>

using namespace Eigen;

class MeasurementModel {

private:
//    VectorXd msr_state; /// measurement (12x1: 6DoF pose + 6 Speed Derivates)
    MatrixXd _msr_cov;   /// measurment covariance (12x12)
    State6DOF _measure;
    tf::TransformListener *_listener;
    tf::StampedTransform _old_transform;
    bool _first_run;
    bool _msg_valid;
    nav_msgs::Odometry _msg, _old_msg; /// current msg arrived from Visual Odometry

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
        _measure._pose = mrs._pose;
        _measure._rotation = mrs._rotation;
        _measure._translational_velocity= mrs._translational_velocity;
        _measure._rotational_velocity= mrs._rotational_velocity;
    }

    State6DOF getMeasureState() { return _measure; }

    void setMeasureCov(MatrixXd& msrcov){ _msr_cov = msrcov; }
    void setMeasureCov(double unc){ _msr_cov = MatrixXd::Identity(12,12) * (unc*unc); }
    void setMeasureCov(double pos_unc, double ori_unc, double lin_unc, double ang_unc) {
        _msr_cov = MatrixXd::Zero(12,12);
        _msr_cov(0,0) = pos_unc*pos_unc;
        _msr_cov(1,1) = pos_unc*pos_unc;
        _msr_cov(2,2) = pos_unc*pos_unc;
        _msr_cov(3,3) = ori_unc*ori_unc;
        _msr_cov(4,4) = ori_unc*ori_unc;
        _msr_cov(5,5) = ori_unc*ori_unc;
        _msr_cov(6,6) = lin_unc*lin_unc;
        _msr_cov(7,7) = lin_unc*lin_unc;
        _msr_cov(8,8) = lin_unc*lin_unc;
        _msr_cov(9,9) = ang_unc*ang_unc;
        _msr_cov(10,10) = ang_unc*ang_unc;
        _msr_cov(11,11) = ang_unc*ang_unc;
    }
    MatrixXd getMeasureCov(){ return _msr_cov; }



    void setMsg(const nav_msgs::Odometry& m);
    bool isMeasureValid() { return _msg_valid; }

    nav_msgs::Odometry getMsg() { return _msg; }
    nav_msgs::Odometry getOldMsg() { return _old_msg; }

    // constructor & destructor -------------------------------------------------------------------
    MeasurementModel(){
        // Sets all values to zero
        _msr_cov = MatrixXd::Zero(12,12);
        _listener = new tf::TransformListener();
        _first_run = true;
        _msg_valid = false;
    }

    ~MeasurementModel(){
        // Resize all values to zero
        _msr_cov.resize(0,0);
        delete _listener;
    }
};

#endif /* ODOMETRY_H_ */

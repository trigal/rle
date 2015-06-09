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
    State6DOF _measure_Delta;
    tf::TransformListener *_listener;
    tf::StampedTransform _old_transform;
    tf::StampedTransform fixed_transform; //fixed transform from ODOM(libviso) to VISUAL_ODOMETRY_X_FORWARD
    bool _msg_valid;
    nav_msgs::Odometry _msg, _old_msg; /// current msg arrived from Visual Odometry

    geometry_msgs::Twist frameSpeed;
    geometry_msgs::Twist oldFrameSpeed;

    class Tracker{

    };

    class Mapper{

    };

public:

    bool _first_run;
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
    void setMeasureState(State6DOF& mrs)
    {
        //        _measure_Delta._pose = mrs._pose;
        //        _measure_Delta._rotation = mrs._rotation;
        //        _measure_Delta._translational_velocity= mrs._translational_velocity;
        //        _measure_Delta._rotational_velocity= mrs._rotational_velocity;
        _measure_Delta.setPose(mrs.getPose());
        _measure_Delta.setRotation(mrs.getRotation());
        _measure_Delta.setTranslationalVelocity(mrs.getTranslationalVelocity());
        _measure_Delta.setRotationalVelocity(mrs.getRotationalVelocity());
    }

    State6DOF getMeasureDeltaState() { return _measure_Delta; }

    //void setMeasureCov(MatrixXd& msrcov){ _msr_cov = msrcov; }
    //void setMeasureCov(double unc){ _msr_cov = MatrixXd::Identity(12,12) * (unc*unc); }
    void setMeasureCov(double position_uncertainty, double orientation_uncertainty, double speed_linear_uncertainty, double speed_angular_uncertainty) {
        _msr_cov = MatrixXd::Zero(12,12);
        _msr_cov(0,0)   = position_uncertainty      * position_uncertainty;
        _msr_cov(1,1)   = position_uncertainty      * position_uncertainty;
        _msr_cov(2,2)   = position_uncertainty      * position_uncertainty;
        _msr_cov(3,3)   = orientation_uncertainty   * orientation_uncertainty;
        _msr_cov(4,4)   = orientation_uncertainty   * orientation_uncertainty;
        _msr_cov(5,5)   = orientation_uncertainty   * orientation_uncertainty;
        _msr_cov(6,6)   = speed_linear_uncertainty  * speed_linear_uncertainty;
        _msr_cov(7,7)   = speed_linear_uncertainty  * speed_linear_uncertainty;
        _msr_cov(8,8)   = speed_linear_uncertainty  * speed_linear_uncertainty;
        _msr_cov(9,9)   = speed_angular_uncertainty * speed_angular_uncertainty;
        _msr_cov(10,10) = speed_angular_uncertainty * speed_angular_uncertainty;
        _msr_cov(11,11) = speed_angular_uncertainty * speed_angular_uncertainty;
    }
    MatrixXd getMeasureCov(){ return _msr_cov; }


    bool getFirstRun() {return _first_run;}
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
    tf::StampedTransform getFixed_transform() const;
    void setFixed_transform(const tf::StampedTransform &value);
};

#endif /* ODOMETRY_H_ */

/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Author:    Dario Limongi                                              *
 *   Email:     dario.limongi@gmail.com                                    *
 *   Date:      02/06/2014                                                 *
 *                                                                         *
 ***************************************************************************/

#ifndef MOTIONMODEL_H_
#define MOTIONMODEL_H_

#include "LayoutComponent.h"
#include "State6DOF.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <boost/assign.hpp>

using namespace Eigen;
using namespace std;

/**
 * This class is used by every particle in order to propagate itself using the
 * defined motion-model.
 * In case there aren't enough datas from Odometry sensors, a decadency motion model
 * will be applied on particle propagation
 */
class MotionModel
{
private:
    MatrixXd error_covariance;  /// 12x12 (double) error covariance matrix
    /// 6DoF + 6 derivate(velocita su ogni DoF)


    /// Random err added to velocity when propagating the pose
    double propagate_translational_percentage_vel_error_x;
    double propagate_translational_percentage_vel_error_y;
    double propagate_translational_percentage_vel_error_z;
    double propagate_rotational_percentage_vel_error;


    /**
     * In case there aren't enough datas from Odometry sensors, a decadency motion model
     * will be applied on particle propagation
     */
    void decadencyMotionModel();

public:
    double propagate_translational_absolute_vel_error_x;
    double propagate_translational_absolute_vel_error_y;
    double propagate_translational_absolute_vel_error_z;
    double propagate_rotational_absolute_vel_error;

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
//    VectorXd propagatePose(VectorXd& particle_state);
    State6DOF propagatePoseWithAbsolute(State6DOF& p_state);                    //old behavior
    State6DOF propagatePoseWithPercentage(State6DOF& p_state);                  //enhanced behavior, but we had troubles with this
    State6DOF propagatePoseWithControl(State6DOF& p_state, State6DOF &control); //used in ITSC 2015 submission
    State6DOF propagatePoseWithControlPercentageAndDeltatime(State6DOF& p_state, State6DOF &control, double deltaTime); //decoupling branch
    State6DOF propagatePoseWithPercentageAndDeltatime(State6DOF& p_state, double deltaTime);                  //enhanced behavior, but we had troubles with this

    /**
     * @brief motionJacobi
     * @param p_state_predicted
     * @return
     */
    MatrixXd motionJacobian(State6DOF &p_state_predicted);

    // getters & setter ------------------------------------------------------------------------
    MatrixXd getErrorCovariance()
    {
        return error_covariance;
    }
    void setErrorCovariance(MatrixXd& err)
    {
        error_covariance = err;
    }
    void setErrorCovariance(double uncertainty)
    {
        error_covariance = MatrixXd::Identity(12, 12) * (uncertainty * uncertainty);
    }
    void setErrorCovariance(double pos_unc, double ori_unc, double lin_unc, double ang_unc)
    {
        error_covariance = MatrixXd::Zero(12, 12);
        error_covariance(0, 0) = pos_unc * pos_unc; //position uncertainty
        error_covariance(1, 1) = pos_unc * pos_unc;
        error_covariance(2, 2) = pos_unc * pos_unc;
        error_covariance(3, 3) = ori_unc * ori_unc; //orientation uncertainty
        error_covariance(4, 4) = ori_unc * ori_unc;
        error_covariance(5, 5) = ori_unc * ori_unc;
        error_covariance(6, 6) = lin_unc * lin_unc; //linear speed uncertainty
        error_covariance(7, 7) = lin_unc * lin_unc;
        error_covariance(8, 8) = lin_unc * lin_unc;
        error_covariance(9, 9) = ang_unc * ang_unc; //angular speed uncertainty
        error_covariance(10, 10) = ang_unc * ang_unc;
        error_covariance(11, 11) = ang_unc * ang_unc;
    }

    void setPropagationError(double translational_absolute_x, double translational_absolute_y, double translational_absolute_z, double rotational_absolute,
                             double translational_percentage_x, double translational_percentage_y, double translational_percentage_z, double rotational_percentage)
    {
        propagate_translational_percentage_vel_error_x          = translational_percentage_x;
        propagate_translational_percentage_vel_error_y          = translational_percentage_y;
        propagate_translational_percentage_vel_error_z          = translational_percentage_z;
        propagate_rotational_percentage_vel_error               = rotational_percentage;

        propagate_translational_absolute_vel_error_x            = translational_absolute_x;
        propagate_translational_absolute_vel_error_y            = translational_absolute_y;
        propagate_translational_absolute_vel_error_z            = translational_absolute_z;
        propagate_rotational_absolute_vel_error                 = rotational_absolute;

        ROS_WARN_STREAM("Calling setPropagationError, *ABSOLUTE* :" << translational_absolute_x << "\t" << translational_absolute_y << "\t" << translational_absolute_z << rotational_absolute );
        ROS_WARN_STREAM("Calling setPropagationError, *PERC.  %* :" << translational_percentage_x << "\t" << translational_percentage_y << "\t" << translational_percentage_z << rotational_percentage );
    }

    // constructor & destructor ----------------------------------------------------------------
    MotionModel(const MatrixXd& cov) : error_covariance(cov) {}
    MotionModel(double inc)
    {
        error_covariance = MatrixXd::Identity(12, 12) * (inc * inc);
    }
    MotionModel()
    {
        // Sets the error covariance to zero
        MatrixXd cov = MatrixXd::Zero(12, 12);
        error_covariance = cov;
    }

    ~MotionModel()
    {
        error_covariance.resize(0, 0);
    }
    double getPropagate_translational_percentage_vel_error_x() const;
    void setPropagate_translational_percentage_vel_error_x(double value);
    double getPropagate_translational_percentage_vel_error_y() const;
    void setPropagate_translational_percentage_vel_error_y(double value);
    double getPropagate_translational_percentage_vel_error_z() const;
    void setPropagate_translational_percentage_vel_error_z(double value);
    double getPropagate_rotational_percentage_vel_error() const;
    void setPropagate_rotational_percentage_vel_error(double value);
};

#endif /* MOTIONMODEL_H_ */

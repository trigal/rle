/*
 * Particle.cpp
 *
 *  Created on: Jun 2, 2014
 *      Author: dario
 */

#include "LayoutComponent.h"
#include "Particle.h"
#include "MotionModel.h"
#include "../Odometry.h"
#include "../LayoutManager.h"
#include <vector>
#include <Eigen/Dense>	//used for pose matrix
#include <Eigen/Core>
using namespace Eigen;
using std::vector;


/**
 * Update every particle's components pose using motion model equations
 */
void Particle::propagateLayoutComponents(){
    vector<LayoutComponent*>::iterator itr;
    for (itr = particle_components.begin(); itr != particle_components.end(); itr++){

        // propagate component pose with motion model equations
        VectorXd pc_state = (*itr)->getComponentState();
        VectorXd new_pose = mtn_model.propagateComponent(pc_state);

        // update pose with predicted one
        (*itr)->setComponentState(new_pose);
    }
}

/** **************************************************************************************************************/
/**
 * @brief Implementation of E.K.F. used for particle Odometry estimation
 * @param particle
 */
void Particle::particleEstimation(Odometry& odometry){

    // initialize variables
    VectorXd stato_t = particle_state;	/// initial state
    MatrixXd E_t = particle_sigma;		/// initial sigma (state error covariance)

    VectorXd stato_t_predetto = VectorXd::Zero(12);	/// predicted state
    MatrixXd E_t_pred = MatrixXd::Zero(12,12);		/// predicted sigma (state error covariance)

    MatrixXd R_t = mtn_model.getErrorCovariance();	/// motion error covariance
    MatrixXd Q_t = odometry.getMeasureCov();	/// measure error covariance

    MatrixXd G_t = MatrixXd::Zero(12,12);	/// motion equations jacobian
    MatrixXd H_t = MatrixXd::Zero(12,12);	/// measure equations jacobian
    MatrixXd K_t = MatrixXd::Zero(12,12);	/// Kalman gain

    // ------- PREDICTION STEP -------
    // calcolo belief predetto:
    stato_t_predetto = mtn_model.propagatePose(stato_t);

    cout << "[delta t] " << LayoutManager::delta_t << endl;
    cout << "[stato_t ]" << endl;
    cout << "   pose: " << stato_t(0) << ", " <<  stato_t(1) <<  ", " << stato_t(2) << " orientation: " << stato_t(3) <<  ", " << stato_t(4) <<  ", " << stato_t(5) << endl;
    cout << "   linear: " << stato_t(6) << ", " <<  stato_t(7) <<  ", " << stato_t(8) << " angular: " << stato_t(9) <<  ", " << stato_t(10) <<  ", " << stato_t(11) << endl;
    cout << "[stato_t_predetto ]" << endl;
    cout << "   pose: " << stato_t_predetto(0) << ", " <<  stato_t_predetto(1) <<  ", " << stato_t_predetto(2) << " orientation: " << stato_t_predetto(3) <<  ", " << stato_t_predetto(4) <<  ", " << stato_t_predetto(5) << endl;
    cout << "   linear: " << stato_t_predetto(6) << ", " <<  stato_t_predetto(7) <<  ", " << stato_t_predetto(8) << " angular: " << stato_t_predetto(9) <<  ", " << stato_t_predetto(10) <<  ", " << stato_t_predetto(11) << endl << endl;

    // applicazione proprietà gaussiane:
    G_t = mtn_model.motionJacobi(stato_t_predetto);
    E_t_pred = G_t * E_t * G_t.transpose() + R_t;

    // ------- UPDATE STEP -------
    // calcolo Kalman gain sull'innovazione:
    H_t = odometry.measurementJacobi(stato_t_predetto);

    MatrixXd temp = H_t * E_t_pred * H_t.transpose() + Q_t;
    K_t = E_t_pred * H_t.transpose() * temp.inverse();
    kalman_gain = K_t; //this value will be used later on score calculation

    // prepare belief calculation values:
    //VectorXd measure_t = visual_odometry.measurePose(stato_t);
    VectorXd measure_t = odometry.getMeasureState();
    VectorXd measure_t_pred = odometry.measurePose(stato_t_predetto); //INTERFACE__VO.GET_MEASURE_FROM_PREDICTED_POSE


    // save temp variables before difference
    double roll = measure_t(3); double roll_pred = measure_t_pred(3);
    double pitch = measure_t(4); double pitch_pred = measure_t_pred(4);
    double yaw = measure_t(5); double yaw_pred = measure_t_pred(5);

    // diference between measures
    VectorXd measure_difference = measure_t - measure_t_pred;

    // DEBUG:
//    cout << endl << "[measure_t orientation: " << measure_t(3) << ", " << measure_t(4) << ", " << measure_t(5) << "] "  << endl;// << measure_t(3) << ", " << endl;<< measure_t(4) << ", " << measure_t(5) << ", " << measure_t(6) << ", " << measure_t(7) << ", " << measure_t(8) << ", " << measure_t(9) << ", " << measure_t(10) << ", " << measure_t(11) << "] "  << endl;
//    cout << "[measure_t_pred orientation: " << measure_t_pred(3) << ", " << measure_t_pred(4) << ", " << measure_t_pred(5) << "] " << endl; // << measure_t_pred(3) << ", " << measure_t_pred(4) << ", " << measure_t_pred(5) << ", " << measure_t_pred(6) << ", " << measure_t_pred(7) << ", " << measure_t_pred(8) << ", " << measure_t_pred(9) << ", " << measure_t_pred(10) << ", " << measure_t_pred(11) << "] "  << endl;
//    cout << "[K_t] " << endl << K_t << endl << endl;
//    cout << "[K_t orientation: " << K_t(3,3) << ", " << K_t(4,4) << ", " << K_t(5,5) << "] " << endl;


    // fix difference between angles
    double diff_roll = Utils::angle_diff(roll,roll_pred);
    double diff_pitch = Utils::angle_diff(pitch, pitch_pred);
    double diff_yaw = Utils::angle_diff(yaw, yaw_pred);

    // fix angles differences
    measure_difference(3) = diff_roll;
    measure_difference(4) = diff_pitch;
    measure_difference(5) = diff_yaw;

    // kalman gain
    VectorXd kalman_per_msr_diff = K_t * measure_difference;
    kalman_per_msr_diff(3) = Utils::normalize_angle(kalman_per_msr_diff(3));
    kalman_per_msr_diff(4) = Utils::normalize_angle(kalman_per_msr_diff(4));
    kalman_per_msr_diff(5) = Utils::normalize_angle(kalman_per_msr_diff(5));

//    cout << "[measure_difference orientation: " << measure_difference(3) << ", " << measure_difference(4) << ", " << measure_difference(5) << "] " << endl;
//    cout << "[K_t * msr. diff. orientation: " << kalman_per_msr_diff(3) << ", " << kalman_per_msr_diff(4) << ", " << kalman_per_msr_diff(5) << "] " << endl;

    // calculate belief
    VectorXd stato_filtrato = stato_t_predetto + kalman_per_msr_diff;
    E_t = (MatrixXd::Identity(12,12) - K_t * H_t) * E_t_pred;

    // normalize angles between -PI and PI because of the sum: stato_t_predetto + (...)
    stato_filtrato(3) = Utils::normalize_angle(stato_filtrato(3));
    stato_filtrato(4) = Utils::normalize_angle(stato_filtrato(4));
    stato_filtrato(5) = Utils::normalize_angle(stato_filtrato(5));

    // update particle values
    particle_state = stato_filtrato;
    particle_sigma = E_t;


    //DEBUG:stampare stato_t_predetto

//    cout << endl << endl;
//    cout << "[stato_t_filtrato ]" << endl;
//    cout << "   pose: " << stato_filtrato(0) << ", " <<  stato_filtrato(1) <<  ", " << stato_filtrato(2) << " orientation: " << stato_filtrato(3) <<  ", " << stato_filtrato(4) <<  ", " << stato_filtrato(5) << endl;
//    cout << "   linear: " << stato_filtrato(6) << ", " <<  stato_filtrato(7) <<  ", " << stato_filtrato(8) << " angular: " << stato_filtrato(9) <<  ", " << stato_filtrato(10) <<  ", " << stato_filtrato(11) << endl;

//    cout << "--------------------------------------------------------------------------------" << endl;
}


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

#include "LayoutComponent.h"
#include "Particle.h"
#include "MotionModel.h"
#include "../MeasurementModel.h"
#include "State6DOF.h"
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
        VectorXd new_pose = particle_mtn_model.propagateComponent(pc_state);

        // update pose with predicted one
        (*itr)->setComponentState(new_pose);
    }
}

/** **************************************************************************************************************/
/**
 * @brief Implementation of E.K.F. used for particle Odometry estimation
 * @param particle
 */
void Particle::particlePoseEstimation(MeasurementModel* odometry, double deltaTime)
{
    // odometry is the measurementModel of the LAYOUTMANAGER (default motion model)
    // check if _first_run is still set.
    if(odometry->getFirstRun())
    {
        ROS_WARN_STREAM("First run in particlePoseEstimation");
        particle_state.setRotationalVelocity(odometry->getMeasureDeltaState().getRotationalVelocity());
        particle_state.setTranslationalVelocity(odometry->getMeasureDeltaState().getTranslationalVelocity());
    }
    else
    {
        ROS_DEBUG_STREAM("particlePoseEstimation: Measure OK");
    }

    /*
     * Here we integrate the new measure from LibViso. The following attempts were proposed.
     * 1. EKF to smooth the odometry messages received, using the uncertainties provided with LibViso2
     * 2. EKF using 'distance' from the path. Not yet implemented, but this will mix the PF weight routine.
     * 3. Straightforward delta integration.
     */

    // cout << "[entering particleEstimation]" << endl;

    // initialize variables
    State6DOF stato_t = particle_state;                                         /// initial state
    MatrixXd E_t = particle_sigma;                                              /// initial sigma (state error covariance)

    State6DOF stato_ut_predetto;
    MatrixXd E_t_pred = MatrixXd::Zero(12,12);                                  /// predicted sigma (state error covariance)

    MatrixXd R_t = particle_mtn_model.getErrorCovariance();                     /// motion error covariance     (mtn_model_position_uncertainty ecc)
    MatrixXd Q_t = odometry->getMeasureCov();                                   /// measure error covariance

    MatrixXd G_t = MatrixXd::Zero(12,12);	                                    /// motion equations jacobian
    MatrixXd H_t = MatrixXd::Zero(12,12);	                                    /// measure equations jacobian
    MatrixXd K_t = MatrixXd::Zero(12,12);	                                    /// Kalman gain

    // ------- PREDICTION STEP -------
    // 1. Absolute
    // stato_ut_predetto = particle_mtn_model.propagatePose(stato_t);

    // 2. Percentage
    //stato_ut_predetto = particle_mtn_model.propagatePoseWithPercentage(stato_t); //percentage

    // 3. With odometry
    //State6DOF deltaFromLibviso = odometry->getMeasureDeltaState();              // _measure from MeasurementModel, the DELTA + speeds
    //stato_ut_predetto = particle_mtn_model.propagatePoseWithControl(stato_t, deltaFromLibviso);

    // 3. With odometry but Percentage
    State6DOF deltaFromLibviso = odometry->getMeasureDeltaState();              // _measure from MeasurementModel, the DELTA + speeds
    stato_ut_predetto = particle_mtn_model.propagatePoseWithPercentageAndDelta(stato_t, deltaTime);

    // Print
    //cout << "[particle_id] " << particle_id << endl;
    //cout << "[delta t]     " << LayoutManager::delta_t << endl;
    stato_t.printState("[stato_t]");
    //State6DOF(odometry->getMsg()).printState("[msg]");
    //State6DOF(odometry->getMeasureDeltaState()).printState("[delta misura]");

    //G_t = particle_mtn_model.motionJacobian(stato_t_predetto); // Check: stato_t al posto di stato_t_predetto
    G_t = particle_mtn_model.motionJacobian(stato_t); // Check: stato_t al posto di stato_t_predetto
    E_t_pred = G_t * E_t * G_t.transpose() + R_t;

    //if (this->getId()==1)
    //{
    //    cout << "\nE_t\n" << E_t << endl << endl;
    //    cout << "\nE_t_pred\n" << E_t_pred << endl << endl;
    //    cout << "\nR_t\n" << R_t << endl << endl;
    //    cout << "\nG_t\n" << G_t << endl << endl;
    //}


    // Check if the measure is valid
    //if(true) // set to TRUE if using "With Odometry"
    if(!odometry->isMeasureValid())
    {
        ROS_WARN_STREAM("Particle.cpp, particlePoseEstimation: Invalid Measure");

        // TODO: smorzare il moto
        particle_state = stato_ut_predetto;

        particle_sigma = E_t_pred;

        //DEBUG:stampa stato_t_predetto
        particle_state.printState("[stato_t_filtrato_prediction_only]");

        //cout << "--------------------------------------------------------------------------------" << endl;

        return;
    }


    // ------- UPDATE STEP -------

    State6DOF delta_measure = odometry->getMeasureDeltaState();                  // differenza tra odometria arrivata, usata per calcolare misura zt
    ROS_ASSERT (delta_measure.getRotation().isUnitary());

//    delta_measure.setRotation(AngleAxisd::Identity());
//    delta_measure.setRotationalVelocity(AngleAxisd::Identity());

    //State6DOF predicted_measure;
    //predicted_measure.setPose(stato_t.getPose() + stato_t.getRotation() * delta_measure.getPose());
    //predicted_measure.setRotation(Eigen::AngleAxisd(delta_measure.getRotation() * stato_t.getRotation()));
    //predicted_measure.setTranslationalVelocity(delta_measure.getTranslationalVelocity());
    //predicted_measure.setRotationalVelocity(delta_measure.getRotationalVelocity());

    State6DOF predicted_measure_zt;
    predicted_measure_zt.setPose(stato_t.getPose() + stato_t.getRotation() * delta_measure.getPose());
    predicted_measure_zt.setRotation(Eigen::AngleAxisd(delta_measure.getRotation() * stato_t.getRotation()));
    ROS_ASSERT (predicted_measure_zt.getRotation().isUnitary());
    predicted_measure_zt.setTranslationalVelocity(delta_measure.getTranslationalVelocity());
    predicted_measure_zt.setRotationalVelocity(delta_measure.getRotationalVelocity());
    ROS_ASSERT (predicted_measure_zt.getRotationalVelocity().isUnitary());

    State6DOF measure_h = odometry->measurePose(stato_ut_predetto);

    // ROS_DEBUG_STREAM ("stato_t_predetto: " << stato_t_predetto.toVectorXd().norm() << " ===== " <<  measure_stato.toVectorXd().norm() << " +++++ " << delta_measure.toVectorXd().norm() );

    stato_ut_predetto.printState("[stato_t_predetto]");
    predicted_measure_zt.printState("[predicted measure]");

    /*nota: stiamo prendendo da libviso il delta, per applicarlo alla particella e confrontarlo con l'update nostro.
     * perché invece non prendiamo direttamente il valore diretto di libviso di POSE e non il delta?
     * risposta: perché nel tempo libviso deriva, il delta probabilmente no, quindi come misura il delta lo prendiamo più affidabile
     *           rispetto alla comulata
     */


//    // CHECK FOR ANGLE-AXIS REPRESENTATION SWITCH
//    Vector3d tmp_check_vect(0.23,-0.41,0.93);
//    double angle_axis_norm = (delta_measure._rotation.angle() * delta_measure._rotation.axis() - measure_delta_stato._rotation.angle() * measure_delta_stato._rotation.axis()).norm();
//    double random_vector_transform_norm = (delta_measure._rotation * tmp_check_vect - measure_delta_stato._rotation * tmp_check_vect).norm();
//    if( fabs(angle_axis_norm / random_vector_transform_norm) > 5)
//    {
//        Q_t.block(3,3,3,3) = MatrixXd::Zero(3,3);
//        ROS_WARN("Angle-axis Rotation representation switch detected");
//    }

//    angle_axis_norm = (delta_measure._rotational_velocity.angle() * delta_measure._rotational_velocity.axis() - measure_delta_stato._rotational_velocity.angle() * measure_delta_stato._rotational_velocity.axis()).norm();
//    random_vector_transform_norm = (delta_measure._rotational_velocity * tmp_check_vect - measure_delta_stato._rotational_velocity * tmp_check_vect).norm();
//    if( fabs(angle_axis_norm / random_vector_transform_norm) > 5)
//    {
//        Q_t.block(9,9,3,3) = MatrixXd::Zero(3,3);
//        ROS_WARN("Angle-axis Rotational Velocity representation switch detected");
//    }

    H_t = odometry->measurementJacobian(stato_ut_predetto);                      //return Identity[12] independently from the parameter

    MatrixXd temp = H_t * E_t_pred * H_t.transpose() + Q_t;
    K_t = E_t_pred * H_t.transpose() * temp.inverse();
    kalman_gain = K_t; //this value will be used later on score calculation

    cout << " qt " << Q_t.norm() << " temp " << temp.norm() << " Ht " << H_t.transpose().norm() << " temp inv " << temp.inverse().norm() << " K " << kalman_gain.norm() << endl;

    // kalman gain
    VectorXd kalman_per_msr_diff =  K_t * (predicted_measure_zt.subtract_vectXd(measure_h));

    //cout << predicted_measure_zt.subtract_vectXd(measure_h) << endl;
    //cout << "kalman_per_msr_diff\n"<<kalman_per_msr_diff<< endl;

    // calculate belief
    State6DOF stato_filtrato = stato_ut_predetto.add_vectXd(kalman_per_msr_diff);
    E_t = (MatrixXd::Identity(12,12) - (K_t * H_t)) * E_t_pred;

    // update particle values
    particle_state = stato_filtrato;
    particle_sigma = E_t;

    //DEBUG:stampa stato_t_predetto
    stato_filtrato.printState("[stato_t_filtrato]");

//    cout << "--------------------------------------------------------------------------------" << endl;
}

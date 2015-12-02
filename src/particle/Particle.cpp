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
#include <Eigen/Dense>  //used for pose matrix
#include <Eigen/Core>
using namespace Eigen;
using std::vector;


/**
 * Update every particle's components pose using motion model equations
 */
void Particle::propagateLayoutComponents()
{
    ROS_DEBUG_STREAM("> Entering propagateLayoutComponents");

    vector<LayoutComponent*>::iterator itr;
    unsigned char component_counter = 0; // preferred over (it - vec.begin())
    for (itr = particle_components.begin(); itr != particle_components.end(); itr++)
    {
        ROS_DEBUG_STREAM("Cycling through component id: " << (int)(component_counter++) << " of " << (*itr)->getComponentId());

        if (dynamic_cast<LayoutComponent_RoadState* >(*itr))
        {
            ROS_DEBUG_STREAM("roadStateComponent detected");
            (*itr)->componentPoseEstimation(); //virtual
        }
        else if (dynamic_cast<LayoutComponent_RoadLane* >(*itr))
        {
            ROS_DEBUG_STREAM("roadLaneComponent detected");
            (*itr)->componentPoseEstimation(); //virtual
        }
        else
        {
            ROS_WARN_STREAM("Unkown component");
        }

        // propagate component pose with motion model equations
        // VectorXd pc_state = (*itr)->getComponentState();
        // VectorXd new_pose = particle_mtn_model.propagateComponent(pc_state);

        // update pose with predicted one
        //(*itr)->setComponentState(new_pose);
    }

    ROS_DEBUG_STREAM("< Exiting propagateLayoutComponents");
}

/** **************************************************************************************************************/
/**
 * @brief     Implementation of E.K.F. used for particle pose propagation
 * @param[in] deltaTime Real deltatime between two consecutive iterations. Not the delta between LIBVISO2 msgs
 * @param[in] odometry  The measurement model, one for all particles/hypotheses
 */
void Particle::particlePoseEstimation(MeasurementModel* odometry, double deltaTimerTime, double deltaOdomTime)
{
    // odometry is the measurementModel of the LAYOUTMANAGER (default motion model)
    // check if _first_run is still set.
    if (odometry->getFirstRun())
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
    MatrixXd E_t_pred = MatrixXd::Zero(12, 12);                                 /// predicted sigma (state error covariance)

    MatrixXd R_t = particle_mtn_model.getErrorCovariance();                     /// motion error covariance     (mtn_model_position_uncertainty ecc)
    MatrixXd Q_t = odometry->getMeasureCov();                                   /// measure error covariance

    MatrixXd G_t = MatrixXd::Zero(12, 12);                                      /// motion equations jacobian
    MatrixXd H_t = MatrixXd::Zero(12, 12);                                      /// measure equations jacobian
    MatrixXd K_t = MatrixXd::Zero(12, 12);                                      /// Kalman gain

    // ------- PREDICTION STEP -------
    /// 1. Absolute
    //stato_ut_predetto = particle_mtn_model.propagatePoseWithAbsolute(stato_t);

    /// 2. Percentage
    //stato_ut_predetto = particle_mtn_model.propagatePoseWithPercentage(stato_t); //percentage

    /// 3. With odometry
    //State6DOF deltaFromLibviso = odometry->getMeasureDeltaState();              // _measure from MeasurementModel, the DELTA + speeds
    //stato_ut_predetto = particle_mtn_model.propagatePoseWithControl(stato_t, deltaFromLibviso);

    /// 3. Tryin again with EKF
    //stato_ut_predetto = particle_mtn_model.propagatePoseWithPercentageAndDeltatime(stato_t, deltaTimerTime);

    /// 4. With odometry but Percentage
    State6DOF deltaFromLibviso = odometry->getMeasureDeltaState();              // _measure from MeasurementModel, the DELTA + speeds
    stato_ut_predetto = particle_mtn_model.propagatePoseWithControlPercentageAndDeltatime(stato_t, deltaFromLibviso, deltaTimerTime);

    // Print
    stato_t.printState("[stato_t]");
    stato_ut_predetto.printState("[stato_t_predetto]");

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
    if (true) // set to TRUE if using "With Odometry"
        //if(!odometry->isMeasureValid())
    {
        ROS_DEBUG_STREAM("Particle.cpp, particlePoseEstimation: Invalid Measure or EKF disabled");

        // TODO: smorzare il moto
        particle_state = stato_ut_predetto;

        particle_sigma = E_t_pred;

        //DEBUG:stampa stato_t_predetto
        particle_state.printState("[stato_t_filtrato_prediction_only]");

        //cout << "--------------------------------------------------------------------------------" << endl;

        /// set roadStateComponent state
        ///
        vector<LayoutComponent*>::iterator itr;
        unsigned char component_counter = 0; // preferred over (it - vec.begin())
        for (itr = particle_components.begin(); itr != particle_components.end(); itr++)
        {

            {
                VectorXd state;
                state = stato_ut_predetto.getPosition();
                ROS_DEBUG_STREAM("Updating the *state (pose)* of roadStateComponent, from particlePoseEstimation:" << state(0) << "\t" << state(1) << "\t" << state(2));

                (*itr)->setComponentState(state); //virtual call
            }
        }


        return;
    }


    // ------- UPDATE STEP -------

    //State6DOF delta_measure = odometry->getMeasureDeltaState();                                            // differenza tra odometria arrivata, usata per calcolare misura zt
    State6DOF delta_measure = odometry->getMeasureDeltaStateScaledWithTime(deltaTimerTime, deltaOdomTime);   // here the measurement is scaled with the timestamps. Needed in decoupling.
    //ROS_ASSERT (delta_measure.getRotation().isUnitary());

//    delta_measure.setRotation(AngleAxisd::Identity());
//    delta_measure.setRotationalVelocity(AngleAxisd::Identity());

    //State6DOF predicted_measure;
    //predicted_measure.setPose(stato_t.getPosition() + stato_t.getRotation() * delta_measure.getPosition());
    //predicted_measure.setRotation(Eigen::AngleAxisd(delta_measure.getRotation() * stato_t.getRotation()));
    //predicted_measure.setTranslationalVelocity(delta_measure.getTranslationalVelocity());
    //predicted_measure.setRotationalVelocity(delta_measure.getRotationalVelocity());

    State6DOF predicted_measure_zt;
    predicted_measure_zt.setPose(stato_t.getPosition() + stato_t.getRotation() * delta_measure.getPosition());
    predicted_measure_zt.setRotation(Eigen::AngleAxisd(delta_measure.getRotation() * stato_t.getRotation()));
    //ROS_ASSERT (predicted_measure_zt.getRotation().isUnitary());
    predicted_measure_zt.setTranslationalVelocity(delta_measure.getTranslationalVelocity());
    predicted_measure_zt.setRotationalVelocity(delta_measure.getRotationalVelocity());
    //ROS_ASSERT (predicted_measure_zt.getRotationalVelocity().isUnitary());

    State6DOF measure_h = odometry->measurePose(stato_ut_predetto);

    // ROS_DEBUG_STREAM ("stato_t_predetto: " << stato_t_predetto.toVectorXd().norm() << " ===== " <<  measure_stato.toVectorXd().norm() << " +++++ " << delta_measure.toVectorXd().norm() );

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

    ROS_DEBUG_STREAM(" qt " << Q_t.norm() << " temp " << temp.norm() << " Ht " << H_t.transpose().norm() << " temp inv " << temp.inverse().norm() << " K " << kalman_gain.norm());

    // kalman gain
    VectorXd kalman_per_msr_diff =  K_t * (predicted_measure_zt.subtract_vectXd(measure_h));

    //cout << predicted_measure_zt.subtract_vectXd(measure_h) << endl;
    //cout << "kalman_per_msr_diff\n"<<kalman_per_msr_diff<< endl;

    // calculate belief
    State6DOF stato_filtrato = stato_ut_predetto.add_vectXd(kalman_per_msr_diff);
    E_t = (MatrixXd::Identity(12, 12) - (K_t * H_t)) * E_t_pred;

    // update particle values
    particle_state = stato_filtrato;
    particle_sigma = E_t;

    //DEBUG:stampa stato_t_predetto
    stato_filtrato.printState("[stato_t_filtrato]");

    //    cout << "--------------------------------------------------------------------------------" << endl;
}

/**
 * @brief Particle::getWayIDHelper
 * @return the wayId inside the LayoutComponent Roadstate. -1 if not found
 *
 * Performs a check inside the components in order to return the WAYID
 * refs #502
 */
int64_t Particle::getWayIDHelper()
{
    for (vector<LayoutComponent*>::iterator it = this->particle_components.begin(); it != this->particle_components.end(); ++it)
    {
        if (dynamic_cast<LayoutComponent_RoadState *>(*it))
        {
            int64_t a=dynamic_cast<LayoutComponent_RoadState *>(*it)->getWay_id();
            return a;
        }
    }

    ROS_ERROR_STREAM("I CAN'T FIND LayoutComponent_RoadState");
    return -1;

    //(dynamic_cast<LayoutComponent_RoadState *>((*particle_itr).getLayoutComponents().at(0)))->getWay_id();
}

/**
 * @brief Particle::getOneWayHelper
 * @return oneway tag value
 *
 * Performs a check inside the components in order to return if the current way
 * has the TAG:ONEWAY and if it is true/false
 */
bool Particle::getOneWayHelper()
{
    for (vector<LayoutComponent*>::iterator it = this->particle_components.begin(); it != this->particle_components.end(); ++it)
    {
        if (dynamic_cast<LayoutComponent_RoadState *>(*it))
        {
            bool a=dynamic_cast<LayoutComponent_RoadState *>(*it)->getOneway();
            return a;
        }
    }

    ROS_ERROR_STREAM("I CAN'T FIND LayoutComponent_RoadState");
    return false;
}



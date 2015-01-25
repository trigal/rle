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
#include "../Odometry.h"
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
    State6DOF stato_t = particle_state;
//    VectorXd stato_t = particle_state;	/// initial state
    MatrixXd E_t = particle_sigma;		/// initial sigma (state error covariance)

    State6DOF stato_t_predetto;
//    VectorXd stato_t_predetto = VectorXd::Zero(12);	/// predicted state
    MatrixXd E_t_pred = MatrixXd::Zero(12,12);		/// predicted sigma (state error covariance)

    MatrixXd R_t = mtn_model.getErrorCovariance();	/// motion error covariance
    MatrixXd Q_t = odometry.getMeasureCov();	/// measure error covariance

    MatrixXd G_t = MatrixXd::Zero(12,12);	/// motion equations jacobian
    MatrixXd H_t = MatrixXd::Zero(12,12);	/// measure equations jacobian
    MatrixXd K_t = MatrixXd::Zero(12,12);	/// Kalman gain

    // ------- PREDICTION STEP -------
    // calcolo belief predetto:
    stato_t_predetto = mtn_model.propagatePose(stato_t);

    cout << "[particle_id] " << particle_id << endl;
    cout << "[delta t]     " << LayoutManager::delta_t << endl;
    cout << "[stato_t]" << endl;
    cout << "       pose: " << stato_t._pose.transpose() << endl << "orientation: " << stato_t._rotation.angle() << " " << stato_t._rotation.axis().transpose() << endl;
    cout << "     linear: " << stato_t._translational_velocity.transpose() << endl << "    angular: " << stato_t._rotational_velocity.angle() << " " << stato_t._rotational_velocity.axis().transpose() << endl << endl;

    cout << "[stato_t_predetto]" << endl;
    cout << "       pose: " << stato_t_predetto._pose.transpose() << endl << "orientation: " << stato_t_predetto._rotation.angle() << " " << stato_t_predetto._rotation.axis().transpose() << endl;
    cout << "     linear: " << stato_t_predetto._translational_velocity.transpose() << endl << "    angular: " << stato_t_predetto._rotational_velocity.angle() << " " << stato_t_predetto._rotational_velocity.axis().transpose() << endl << endl;

    // applicazione proprietà gaussiane:
    G_t = mtn_model.motionJacobian(stato_t_predetto);
    E_t_pred = G_t * E_t * G_t.transpose() + R_t;


    // ------- UPDATE STEP -------
    // calcolo Kalman gain sull'innovazione:
    // prepare belief calculation values:
    //VectorXd measure_t = visual_odometry.measurePose(stato_t);
    State6DOF measure_t = odometry.getMeasureState();
    State6DOF measure_t_pred = odometry.measurePose(stato_t_predetto); //INTERFACE__VO.GET_MEASURE_FROM_PREDICTED_POSE

    // CHECK FOR ANGLE-AXIS REPRESENTATION SWITCH
    Vector3d tmp_check_vect(0.23,-0.41,0.93);
    double angle_axis_norm = (measure_t._rotation.angle() * measure_t._rotation.axis() - measure_t_pred._rotation.angle() * measure_t_pred._rotation.axis()).norm();
    double random_vector_transform_norm = (measure_t._rotation * tmp_check_vect - measure_t_pred._rotation * tmp_check_vect).norm();
    if( fabs(angle_axis_norm / random_vector_transform_norm) > 5)
        Q_t.block(3,3,3,3) = MatrixXd::Zero(3,3);

    angle_axis_norm = (measure_t._rotational_velocity.angle() * measure_t._rotational_velocity.axis() - measure_t_pred._rotational_velocity.angle() * measure_t_pred._rotational_velocity.axis()).norm();
    random_vector_transform_norm = (measure_t._rotational_velocity * tmp_check_vect - measure_t_pred._rotational_velocity * tmp_check_vect).norm();
    if( fabs(angle_axis_norm / random_vector_transform_norm) > 5)
        Q_t.block(9,9,3,3) = MatrixXd::Zero(3,3);

    H_t = odometry.measurementJacobian(stato_t_predetto);

    MatrixXd temp = H_t * E_t_pred * H_t.transpose() + Q_t;
    K_t = E_t_pred * H_t.transpose() * temp.inverse();
    kalman_gain = K_t; //this value will be used later on score calculation

//    // diference between measures
//    State6DOF measure_difference = measure_t - measure_t_pred;

    // DEBUG:
//    cout << endl << "[measure_t orientation: " << measure_t(3) << ", " << measure_t(4) << ", " << measure_t(5) << "] "  << endl;// << measure_t(3) << ", " << endl;<< measure_t(4) << ", " << measure_t(5) << ", " << measure_t(6) << ", " << measure_t(7) << ", " << measure_t(8) << ", " << measure_t(9) << ", " << measure_t(10) << ", " << measure_t(11) << "] "  << endl;
//    cout << "[measure_t_pred orientation: " << measure_t_pred(3) << ", " << measure_t_pred(4) << ", " << measure_t_pred(5) << "] " << endl; // << measure_t_pred(3) << ", " << measure_t_pred(4) << ", " << measure_t_pred(5) << ", " << measure_t_pred(6) << ", " << measure_t_pred(7) << ", " << measure_t_pred(8) << ", " << measure_t_pred(9) << ", " << measure_t_pred(10) << ", " << measure_t_pred(11) << "] "  << endl;
//    cout << "[K_t] " << endl << K_t << endl << endl;
//    cout << "[K_t orientation: " << K_t(3,3) << ", " << K_t(4,4) << ", " << K_t(5,5) << "] " << endl;


    // kalman gain
    VectorXd kalman_per_msr_diff = K_t * measure_t.subtract(measure_t_pred);
//    cout << "[measure_difference orientation: " << measure_difference(3) << ", " << measure_difference(4) << ", " << measure_difference(5) << "] " << endl;
//    cout << "[K_t * msr. diff. orientation: " << kalman_per_msr_diff(3) << ", " << kalman_per_msr_diff(4) << ", " << kalman_per_msr_diff(5) << "] " << endl;

    // calculate belief
    State6DOF stato_filtrato = stato_t_predetto.addVectorXd(kalman_per_msr_diff);
    E_t = (MatrixXd::Identity(12,12) - K_t * H_t) * E_t_pred;

    // update particle values
    particle_state = stato_filtrato;
    particle_sigma = E_t;

    //DEBUG:stampare stato_t_predetto
    cout << "[stato_t_filtrato]" << endl;
    cout << "       pose: " << stato_filtrato._pose.transpose() << endl << "orientation: " << stato_filtrato._rotation.angle() << " " << stato_filtrato._rotation.axis().transpose() << endl;
    cout << "     linear: " << stato_filtrato._translational_velocity.transpose() << endl << "    angular: " << stato_filtrato._rotational_velocity.angle() << " " << stato_filtrato._rotational_velocity.axis().transpose() << endl << endl;

    cout << "--------------------------------------------------------------------------------" << endl;
}


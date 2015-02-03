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
void Particle::particleEstimation(Odometry* odometry){

    // initialize variables
    State6DOF stato_t = particle_state; /// initial state
    MatrixXd E_t = particle_sigma;		/// initial sigma (state error covariance)

    State6DOF stato_t_predetto;
    MatrixXd E_t_pred = MatrixXd::Zero(12,12);		/// predicted sigma (state error covariance)

    MatrixXd R_t = particle_mtn_model.getErrorCovariance();	/// motion error covariance
    MatrixXd Q_t = odometry->getMeasureCov();	/// measure error covariance

    MatrixXd G_t = MatrixXd::Zero(12,12);	/// motion equations jacobian
    MatrixXd H_t = MatrixXd::Zero(12,12);	/// measure equations jacobian
    MatrixXd K_t = MatrixXd::Zero(12,12);	/// Kalman gain

    // ------- PREDICTION STEP -------
    // calcolo belief predetto:
    stato_t_predetto = particle_mtn_model.propagatePose(stato_t);

    cout << "[particle_id] " << particle_id << endl;
    cout << "[delta t]     " << LayoutManager::delta_t << endl;
    cout << "[stato_t]" << endl;
    cout << "       pose: " << stato_t._pose.transpose() << endl << "orientation: " << stato_t._rotation.angle() << " " << stato_t._rotation.axis().transpose() << endl;
    cout << "     linear: " << stato_t._translational_velocity.transpose() << endl << "    angular: " << stato_t._rotational_velocity.angle() << " " << stato_t._rotational_velocity.axis().transpose() << endl << endl;

    cout << "[stato_t_predetto]" << endl;
    cout << "       pose: " << stato_t_predetto._pose.transpose() << endl << "orientation: " << stato_t_predetto._rotation.angle() << " " << stato_t_predetto._rotation.axis().transpose() << endl;
    cout << "     linear: " << stato_t_predetto._translational_velocity.transpose() << endl << "    angular: " << stato_t_predetto._rotational_velocity.angle() << " " << stato_t_predetto._rotational_velocity.axis().transpose() << endl << endl;

    // applicazione proprietà gaussiane:
    G_t = particle_mtn_model.motionJacobian(stato_t_predetto); // Check: stato_t al posto di stato_t_predetto
    E_t_pred = G_t * E_t * G_t.transpose() + R_t;

    // ------- UPDATE STEP -------
    State6DOF measure_t = odometry->getMeasureState();                  // differenza tra odometria arrivata
    State6DOF delta_measure = measure_t.subtract_state6DOF(old_measure);
    old_measure = measure_t;

    State6DOF delta_stato = stato_t_predetto.subtract_state6DOF(stato_t);
    State6DOF measure_delta_stato = odometry->measurePose(delta_stato); //odometry->measurePose(stato_t_predetto); //  stato_t-1_predett - stato_t_predetto


    // CHECK FOR ANGLE-AXIS REPRESENTATION SWITCH
    Vector3d tmp_check_vect(0.23,-0.41,0.93);
    double angle_axis_norm = (delta_measure._rotation.angle() * delta_measure._rotation.axis() - measure_delta_stato._rotation.angle() * measure_delta_stato._rotation.axis()).norm();
    double random_vector_transform_norm = (delta_measure._rotation * tmp_check_vect - measure_delta_stato._rotation * tmp_check_vect).norm();
    if( fabs(angle_axis_norm / random_vector_transform_norm) > 5)
    {
        Q_t.block(3,3,3,3) = MatrixXd::Zero(3,3);
        ROS_WARN("Angle-axis Rotation representation switch detected");
    }

    angle_axis_norm = (delta_measure._rotational_velocity.angle() * delta_measure._rotational_velocity.axis() - measure_delta_stato._rotational_velocity.angle() * measure_delta_stato._rotational_velocity.axis()).norm();
    random_vector_transform_norm = (delta_measure._rotational_velocity * tmp_check_vect - measure_delta_stato._rotational_velocity * tmp_check_vect).norm();
    if( fabs(angle_axis_norm / random_vector_transform_norm) > 5)
    {
        Q_t.block(9,9,3,3) = MatrixXd::Zero(3,3);
        ROS_WARN("Angle-axis Rotational Velocity representation switch detected");
    }

    H_t = odometry->measurementJacobian(stato_t_predetto);

    MatrixXd temp = H_t * E_t_pred * H_t.transpose() + Q_t;
    K_t = E_t_pred * H_t.transpose() * temp.inverse();
    kalman_gain = K_t; //this value will be used later on score calculation

    // kalman gain
    VectorXd kalman_per_msr_diff = K_t * delta_measure.subtract_vect(measure_delta_stato);

    // calculate belief
    State6DOF stato_filtrato = stato_t_predetto.addVectorXd(kalman_per_msr_diff);
    E_t = (MatrixXd::Identity(12,12) - (K_t * H_t)) * E_t_pred;
    // update particle values
    particle_state = stato_filtrato;
    particle_sigma = E_t;

    //DEBUG:stampa stato_t_predetto
    cout << "[stato_t_filtrato]" << endl;
    cout << "       pose: " << stato_filtrato._pose.transpose() << endl << "orientation: " << stato_filtrato._rotation.angle() << " " << stato_filtrato._rotation.axis().transpose() << endl;
    cout << "     linear: " << stato_filtrato._translational_velocity.transpose() << endl << "    angular: " << stato_filtrato._rotational_velocity.angle() << " " << stato_filtrato._rotational_velocity.axis().transpose() << endl << endl;

    cout << "--------------------------------------------------------------------------------" << endl;
}


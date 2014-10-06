/*
 * LayoutManager.cpp
 *
 *  Created on: May 25, 2014
 *      Author: dario
 */

#include "Odometry.h"
#include "Utils.h"
#include "LayoutManager.h"
#include "particle/Particle.h"
#include "particle/LayoutComponent_Building.h"
#include "particle/LayoutComponent.h"
#include <vector>	//used for vector
#include <Eigen/Core>
#include <Eigen/Dense>	//used for motion threshold matrix
#include <iostream>
using Eigen::ArrayXd;
using Eigen::MatrixXd;
using namespace std;

/**
 * Initialize static member value for C++ compilation
 */
double LayoutManager::delta_t = 1;

/**
 * Check if car has moved or not by confronting odometry matrix and motion threshold matrix
 * @return true if car has moved beyond the the threshold matrix
 */
bool LayoutManager::checkHasMoved(){
	//MatrixXd diff_matrix = (visual_odometry.getOdometry().array() > motion_threshold).cast<double>();
//	MatrixXd diff_matrix = MatrixXd::Ones(12,12);
//	return diff_matrix.count() > 0; //every position over the threshold will count as 1

    return true;
}

/** **************************************************************************************************************/
/**
 * Estimate particles' components using particle filter
 */
void LayoutManager::componentsEstimation(){
    // STEP 1: SAMPLING (PREDICT COMPONENTS POSES)
    sampling();

    // STEP 2: PERTURBATE COMPONENT POSES
    componentsPerturbation();

    // STEP 3: WEIGHT LAYOUT-COMPONENTS
    calculateLayoutComponentsWeight();
}

/**
 * PARTICLE FILTER, STEP 1:
 * cicle through all the particles, and call their function "propagate-components"
 */
void LayoutManager::sampling(){
	vector<Particle>::iterator itr;
	for( itr = current_layout.begin(); itr != current_layout.end(); itr++ ){
		cout << "--- Propagating components of particle, ID: " << itr->getId() << " ---"<< endl;
        itr->propagateLayoutComponents();
	}
}

/**
 * PARTICLE FILTER, STEP 2:
 * @brief LayoutManager::componentsPerturbation
 */
void LayoutManager::componentsPerturbation(){

    // first, iterate over all particles of 'current_layout'
    for(int i=0; i<current_layout.size(); i++){
        Particle p = current_layout.at(i);
        vector<LayoutComponent*> vec = p.getLayoutComponents();

        // second, iterate over all layout-components of current 'particle'
        for(int j=0; j<vec.size(); j++){
            LayoutComponent* lc = vec.at(j);
            lc->componentPerturbation();
        }
    }
}

/**
 * PARTICLE FILTER, STEP 3:
 * @brief LayoutManager::calculateLayoutComponentsWeight
 *
 * andiamo a vedere quanto bene fitta la componente della singola particella nella realtà
 */
void LayoutManager::calculateLayoutComponentsWeight(){
    // first, iterate over all particles of 'current_layout'
    for(int i=0; i<current_layout.size(); i++){
        Particle p = current_layout.at(i);
        vector<LayoutComponent*> vec = p.getLayoutComponents();

        // second, iterate over all layout-components of current 'particle'
        for(int j=0; j<vec.size(); j++){
            LayoutComponent* lc = vec.at(j);
            lc->calculateWeight();
        }
    }
}
/** **************************************************************************************************************/


/** **************************************************************************************************************/
/**
 * @brief Implementation of E.K.F. used for particle Odometry estimation
 * @param particle
 */
void LayoutManager::particleEstimation(Particle & particle){

	// initialize variables
	VectorXd stato_t = particle.getParticleState();	/// initial state
	MatrixXd E_t = particle.getParticleSigma();		/// initial sigma (state error covariance)

	VectorXd stato_t_predetto = VectorXd::Zero(12);	/// predicted state
	MatrixXd E_t_pred = MatrixXd::Zero(12,12);		/// predicted sigma (state error covariance)

	MatrixXd R_t = particle.mtn_model.getErrorCovariance();	/// motion error covariance
    MatrixXd Q_t = msr_cov;	/// measure error covariance

	MatrixXd G_t = MatrixXd::Zero(12,12);	/// motion equations jacobian
	MatrixXd H_t = MatrixXd::Zero(12,12);	/// measure equations jacobian
	MatrixXd K_t = MatrixXd::Zero(12,12);	/// Kalman gain

    // ------- PREDICTION STEP -------
    // calcolo belief predetto:
    stato_t_predetto = particle.mtn_model.propagatePose(stato_t);

    cout << "[delta t] " << delta_t << endl;
    cout << "[stato_t ]" << endl;
    cout << "   pose: " << stato_t(0) << ", " <<  stato_t(1) <<  ", " << stato_t(2) << " orientation: " << stato_t(3) <<  ", " << stato_t(4) <<  ", " << stato_t(5) << endl;
    cout << "   linear: " << stato_t(6) << ", " <<  stato_t(7) <<  ", " << stato_t(8) << " angular: " << stato_t(9) <<  ", " << stato_t(10) <<  ", " << stato_t(11) << endl;
    cout << "[stato_t_predetto ]" << endl;
    cout << "   pose: " << stato_t_predetto(0) << ", " <<  stato_t_predetto(1) <<  ", " << stato_t_predetto(2) << " orientation: " << stato_t_predetto(3) <<  ", " << stato_t_predetto(4) <<  ", " << stato_t_predetto(5) << endl;
    cout << "   linear: " << stato_t_predetto(6) << ", " <<  stato_t_predetto(7) <<  ", " << stato_t_predetto(8) << " angular: " << stato_t_predetto(9) <<  ", " << stato_t_predetto(10) <<  ", " << stato_t_predetto(11) << endl << endl;

    // applicazione proprietà gaussiane:
    G_t = particle.mtn_model.motionJacobi(stato_t_predetto);
	E_t_pred = G_t * E_t * G_t.transpose() + R_t;

    // ------- UPDATE STEP -------
    // calcolo Kalman gain sull'innovazione:
    H_t = visual_odometry.measurementJacobi(stato_t_predetto);

	MatrixXd temp = H_t * E_t_pred * H_t.transpose() + Q_t;
	K_t = E_t_pred * H_t.transpose() * temp.inverse();
    particle.setKalmanGain(K_t);

    // prepare belief calculation values:
    //VectorXd measure_t = visual_odometry.measurePose(stato_t);
    VectorXd measure_t = msr_state;
    VectorXd measure_t_pred = visual_odometry.measurePose(stato_t_predetto); //INTERFACE__VO.GET_MEASURE_FROM_PREDICTED_POSE


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
    double diff_roll = angle_diff(roll,roll_pred);
    double diff_pitch = angle_diff(pitch, pitch_pred);
    double diff_yaw = angle_diff(yaw, yaw_pred);

    // fix angles differences
    measure_difference(3) = diff_roll;
    measure_difference(4) = diff_pitch;
    measure_difference(5) = diff_yaw;

    // kalman gain
    VectorXd kalman_per_msr_diff = K_t * measure_difference;
    kalman_per_msr_diff(3) = normalize_angle(kalman_per_msr_diff(3));
    kalman_per_msr_diff(4) = normalize_angle(kalman_per_msr_diff(4));
    kalman_per_msr_diff(5) = normalize_angle(kalman_per_msr_diff(5));

//    cout << "[measure_difference orientation: " << measure_difference(3) << ", " << measure_difference(4) << ", " << measure_difference(5) << "] " << endl;
//    cout << "[K_t * msr. diff. orientation: " << kalman_per_msr_diff(3) << ", " << kalman_per_msr_diff(4) << ", " << kalman_per_msr_diff(5) << "] " << endl;

    // calculate belief
    VectorXd stato_filtrato = stato_t_predetto + kalman_per_msr_diff;
	E_t = (MatrixXd::Identity(12,12) - K_t * H_t) * E_t_pred;

    // normalize angles between -PI and PI because of the sum: stato_t_predetto + (...)
    stato_filtrato(3) = normalize_angle(stato_filtrato(3));
    stato_filtrato(4) = normalize_angle(stato_filtrato(4));
    stato_filtrato(5) = normalize_angle(stato_filtrato(5));

    // update particle values
    particle.setParticleState(stato_filtrato);
	particle.setParticleSigma(E_t);


    //DEBUG:stampare stato_t_predetto

//    cout << endl << endl;
//    cout << "[stato_t_filtrato ]" << endl;
//    cout << "   pose: " << stato_filtrato(0) << ", " <<  stato_filtrato(1) <<  ", " << stato_filtrato(2) << " orientation: " << stato_filtrato(3) <<  ", " << stato_filtrato(4) <<  ", " << stato_filtrato(5) << endl;
//    cout << "   linear: " << stato_filtrato(6) << ", " <<  stato_filtrato(7) <<  ", " << stato_filtrato(8) << " angular: " << stato_filtrato(9) <<  ", " << stato_filtrato(10) <<  ", " << stato_filtrato(11) << endl;

//    cout << "--------------------------------------------------------------------------------" << endl;
}
/** **************************************************************************************************************/

/** **************************************************************************************************************/
/**
 * FORMULA CALCOLO SCORE
 *
 * Cardinalità unaria (Particella presa INDIVIDUALMENTE)
 *  1- Kalman gain sulla pose della particella
 *  2- Somma dei WEIGHT delle varie componenti della particella (ottenuti dal filtraggio a particelle)
 *
 * Cardinalità >= 2
 *  1- plausibilità di esistenza tra le varie componenti di stesso tipo (due building sovrapposti ecc.) nella stessa particella
 *  2- plausibilità di esistenza tra componenti di diverso tipo (building sovrapposto a una macchina ecc.) nella stessa particella
 *
 * Nessuna particella verrà eliminata durante il procedimento di calcolo dello score,
 * essa sarà mantenuta in vita nonostante lo score sia basso.
 * In questo modo si evita la possibilità di eliminare dal particle-set ipotesi plausibili che abbiano ricevuto
 * uno score di valore basso per motivi di natura diversa.
 *
 */
void LayoutManager::calculateScore(){
    vector<Particle>::iterator p_itr;
    for( p_itr = current_layout.begin(); p_itr != current_layout.end(); p_itr++ ){

        // calculate summatory of all components weights
        double components_weight = 0;
        vector<LayoutComponent*> vec = p_itr->getLayoutComponents();
        vector<LayoutComponent*>::iterator pc_itr;
        for(pc_itr = vec.begin(); pc_itr != vec.end(); pc_itr++){
            components_weight += (*pc_itr)->getComponentWeight();
        }

        // calculate unary score
        MatrixXd unary_score = p_itr->getKalmanGain();
        //cout << "Particle ID: " << p_itr->getId() << ", unary score: "<< unary_score << endl;

        // calculate binary score
        MatrixXd binary_score = MatrixXd::Zero(12,12);

        // calculate particle score
        MatrixXd particle_score = unary_score * binary_score;
        //p_itr->setParticleScore(particle_score);
    }
}
/** **************************************************************************************************************/

vector<Particle> LayoutManager::layoutEstimation(){
	// Check if car has moved, if it has moved then estimate new layout
	if( checkHasMoved() )
	{
		// ----------------- predict and update layout poses using E.K.F ----------------- //
		vector<Particle>::iterator particle_itr;
		for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ ){
			this->particleEstimation(*particle_itr);
		}
		// ------------------------------------------------------------------------------- //

		// -------------- sampling + perturbation + weight layout-components ------------- //
        this->componentsEstimation();
		// ------------------------------------------------------------------------------- //

		// ------------------------------ calculate score -------------------------------- //
        this->calculateScore();
		// ------------------------------------------------------------------------------- //


        if(new_detections)
		{
//			Add new candidate-components ----------------------------------------------------- //
//			(1) given N new candidate-layout-components duplicate 2^N particles and those particles to them
//			(2) calculate score
//			(3) resample all combination
		}
	}
	else
	{
		cout << endl <<  "Not moved!" << endl;
        //TODO: calculate score again, with new detections (if available)

	}


	return current_layout;
}


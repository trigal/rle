/*
 * LayoutManager.cpp
 *
 *  Created on: May 25, 2014
 *      Author: dario
 */

#include "VisualOdometry.h"
#include "LayoutManager.h"
#include "particle/Particle.h"
#include <vector>	//used for vector
#include <Eigen/Core>
#include <Eigen/Dense>	//used for motion threshold matrix
#include <iostream>
using Eigen::ArrayXd;
using Eigen::MatrixXd;
using namespace std;

/**
 * Check if car has moved or not by confronting odometry matrix and motion threshold matrix
 * @return true if car has moved beyond the the threshold matrix
 */
bool LayoutManager::checkHasMoved(){
	//MatrixXd diff_matrix = (visual_odometry.getOdometry().array() > motion_threshold).cast<double>();
	MatrixXd diff_matrix = MatrixXd::Ones(12,12);
	return diff_matrix.count() > 0; //every position over the threshold will count as 1
}

/**
 *  cicle through all the particles, and call their function "propagate-components"
 *  that propagate every component of the particle
 */
void LayoutManager::sampling(){
	vector<Particle>::iterator itr;
	for( itr = current_layout.begin(); itr != current_layout.end(); itr++ ){
		cout << "--- Propagating components of particle, ID: " << itr->getId() << " ---"<< endl;
		itr->propagateParticleComponents();
	}
}

void LayoutManager::resampling(){

}

void LayoutManager::componentsPerturbation(){

}


void LayoutManager::particleEstimation(Particle& particle){

	// initialize variables
	VectorXd stato_t = particle.getParticleState();	/// initial state
	MatrixXd E_t = particle.getParticleSigma();		/// initial sigma (state error covariance)

	VectorXd stato_t_predetto = VectorXd::Zero(12);	/// predicted state
	MatrixXd E_t_pred = MatrixXd::Zero(12,12);		/// predicted sigma (state error covariance)

	MatrixXd R_t = particle.mtn_model.getErrorCovariance();	/// motion error covariance
	MatrixXd Q_t = visual_odometry.getErrorCovariance();	/// measure error covariance

	MatrixXd G_t = MatrixXd::Zero(12,12);	/// motion equations jacobian
	MatrixXd H_t = MatrixXd::Zero(12,12);	/// measure equations jacobian
	MatrixXd K_t = MatrixXd::Zero(12,12);	/// Kalman gain

//	%% ------- PREDICTION STEP -------
//
//	% calcolo belief predetto:
//	stato_t_predetto = transiz_stato(stato_prec, controllo, zeros(3,3), b);
	stato_t_predetto = particle.propagateParticlePose();

//	% applicazione proprietà gaussiane:
//	G_t = jacobiana_g(stato_prec, controllo, b);
	G_t = particle.mtn_model.motionJacobi(stato_t_predetto);

//	E_t_pred = G_t * E_t * G_t' + R_t;
	E_t_pred = G_t * E_t * G_t.transpose() + R_t;


//  %% ------- UPDATE STEP -------
//  % calcolo Kalman gain sull'innovazione:
//  H_t = jacobiana_h(stato_t_predetto, h, f, d);
	H_t = particle.mtn_model.measurementJacobi(stato_t_predetto);

//  K_t = E_t_pred * H_t' * inv(H_t * E_t_pred * H_t' + Q_t);
	MatrixXd temp = H_t * E_t_pred * H_t.transpose() + Q_t;
	K_t = E_t_pred * H_t.transpose() * temp.inverse();

//  % calcolo belief:
//  stato_t = stato_t_predetto' + K_t * (misura_t - eq_misura(stato_t_predetto, zeros(4,4), h, f, d))';
	VectorXd measure_t = visual_odometry.measurePose(stato_t);
	VectorXd measure_t_pred = visual_odometry.measurePose(stato_t_predetto);
	stato_t = stato_t_predetto + K_t * (measure_t - measure_t_pred);

//  E_t = (eye(3) - K_t * H_t) * E_t_pred;
	E_t = (MatrixXd::Identity(12,12) - K_t * H_t) * E_t_pred;

	// update particle values
	particle.setParticleState(stato_t);
	particle.setParticleSigma(E_t);
}

void LayoutManager::componentsEstimation(){
	// STEP 1: SAMPLING (PREDICT COMPONENTS POSES)
	sampling();

	// STEP 2: PERTURBATE COMPONENT POSES
	componentsPerturbation();

	// STEP 3: WEIGHT LAYOUT-COMPONENTS
	vector<Particle>::iterator itr;
	for( itr = current_layout.begin(); itr != current_layout.end(); itr++ ){
		itr->calculateComponentsScore();
	}

	// **** RESAMPLING ****
	if(is_new_detection){
		resampling();
	}
}

vector<Particle> LayoutManager::layoutEstimation(){
	// Check if car has moved, if it has moved then estimate new layout
	if( checkHasMoved() )
	{
		cout << endl << "Has moved!" << endl;

		// ----------------- predict and update layout poses using E.K.F ----------------- //
		vector<Particle>::iterator particle_itr;
		for( particle_itr = current_layout.begin(); particle_itr != current_layout.end(); particle_itr++ ){
			this->particleEstimation(*particle_itr);
		}
		// ------------------------------------------------------------------------------- //

		// -------------- sampling + perturbation + weight layout-components ------------- //
		//this->componentsEstimation();
		// ------------------------------------------------------------------------------- //

		// ------------------------------ calculate score -------------------------------- //
		//this->calculateScore();
		// ------------------------------------------------------------------------------- //


		if(is_new_detection)
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
		//TODO: calculate score again, with new detections
	}


	return current_layout;
}


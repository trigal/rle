/*
 * LayoutManager.cpp
 *
 *  Created on: May 25, 2014
 *      Author: dario
 */

#include "Odometry.h"
#include "LayoutManager.h"
#include "particle/Particle.h"
#include <vector>	//used for vector
#include <Eigen/Core>
#include <Eigen/Dense>	//used for motion threshold matrix
#include <iostream>
using Eigen::ArrayXd;
using Eigen::MatrixXd;
using namespace std;
const double PI = 3.14159265358979323846;
const double PI_TIMES_2 = 2.0 * PI;

double normalize_augusto(double z)
{
  return atan2(sin(z),cos(z));
}
double angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize_augusto(a);
  b = normalize_augusto(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

double angleDiff(double firstAngle, double secondAngle)
{
    double difference = secondAngle - firstAngle;
    while (difference < -PI) difference += PI_TIMES_2;
    while (difference > PI) difference -= PI_TIMES_2;
    return difference;
}

/**
 * Computes the unoriented smallest difference between two angles.
 * The angles are assumed to be normalized to the range [-Pi, Pi]. The result will be in the range [0, Pi].
 *
 * @param ang1 the angle of one vector (in [-Pi, Pi] )
 * @param ang2 	the angle of the other vector (in range [-Pi, Pi] )
 * @return the angle (in radians) between the two vectors (in range [0, Pi] )
 */
double diff_2(double ang1, double ang2) /// from GEOS library
{
    double delAngle;

    if (ang1 < ang2) {
        delAngle = ang2 - ang1;
    } else {
        delAngle = ang1 - ang2;
    }

    if (delAngle > PI) {
        delAngle = (2 * PI) - delAngle;
    }

    return delAngle;
}

/**
 * Computes the normalized value of an angle, which is the equivalent angle in the range ( -Pi, Pi ].
 * @param angle	the angle to normalize
 * @return an equivalent angle in the range (-Pi, Pi]
 */
double normalize_2(double angle)
{
    while (angle > PI)
        angle -= PI_TIMES_2;
    while (angle <= -PI)
        angle += PI_TIMES_2;
    return angle;
}


/**
 * @brief LayoutManager::setParticlesDelta
 * @param delta
 */
void LayoutManager::setParticlesDelta(double delta){
    vector<Particle>::iterator itr;
    for (itr = current_layout.begin(); itr != current_layout.end(); itr++){
        Particle p = *itr;
        p.mtn_model.delta_t = delta;
    }
}

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


void LayoutManager::particleEstimation(Particle & particle){

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

    // ------- PREDICTION STEP -------
    // calcolo belief predetto:
    stato_t_predetto = particle.mtn_model.propagatePose(stato_t);

    // applicazione proprietà gaussiane:
	G_t = particle.mtn_model.motionJacobi(stato_t_predetto);
	E_t_pred = G_t * E_t * G_t.transpose() + R_t;

    // ------- UPDATE STEP -------
    // calcolo Kalman gain sull'innovazione:
    H_t = visual_odometry.measurementJacobi(stato_t_predetto);

	MatrixXd temp = H_t * E_t_pred * H_t.transpose() + Q_t;
	K_t = E_t_pred * H_t.transpose() * temp.inverse();

    // prepare belief calculation values:
    //VectorXd measure_t = visual_odometry.measurePose(stato_t);
    VectorXd measure_t = visual_odometry.getCurrentMeasurement();
    VectorXd measure_t_pred = visual_odometry.measurePose(stato_t_predetto); //INTERFACE__VO.GET_MEASURE_FROM_PREDICTED_POSE

    // save temp variables before difference
    double roll = measure_t(3); double roll_pred = measure_t_pred(3);
    double pitch = measure_t(4); double pitch_pred = measure_t_pred(4);
    double yaw = measure_t(5); double yaw_pred = measure_t_pred(5);

    // diference between measures
    VectorXd measure_difference = measure_t - measure_t_pred;

    // DEBUG:
    cout << endl << "[measure_t orientation: " << measure_t(3) << ", " << measure_t(4) << ", " << measure_t(5) << "] "  << endl;// << measure_t(3) << ", " << endl;<< measure_t(4) << ", " << measure_t(5) << ", " << measure_t(6) << ", " << measure_t(7) << ", " << measure_t(8) << ", " << measure_t(9) << ", " << measure_t(10) << ", " << measure_t(11) << "] "  << endl;
    cout << "[measure_t_pred orientation: " << measure_t_pred(3) << ", " << measure_t_pred(4) << ", " << measure_t_pred(5) << "] " << endl; // << measure_t_pred(3) << ", " << measure_t_pred(4) << ", " << measure_t_pred(5) << ", " << measure_t_pred(6) << ", " << measure_t_pred(7) << ", " << measure_t_pred(8) << ", " << measure_t_pred(9) << ", " << measure_t_pred(10) << ", " << measure_t_pred(11) << "] "  << endl;
//    cout << "[K_t] " << endl << K_t << endl << endl;
    cout << "[K_t orientation: " << K_t(3,3) << ", " << K_t(4,4) << ", " << K_t(5,5) << "] " << endl;


    // fix difference between angles
    double diff_roll = angle_diff( roll_pred, roll);
    double diff_pitch = angle_diff( pitch_pred, pitch );
    double diff_yaw = angle_diff( yaw_pred, yaw );

//    cout << "[norm orient: " << normalize_2(roll) << ", "<< normalize_2(pitch) << ", "<< normalize_2(yaw) << "] " << endl;
//    cout << "[orient_prec: " << normalize_2(roll_pred) << ", "<< normalize_2(pitch_pred) << ", "<< normalize_2(yaw_pred) << "] [norm orient: " << normalize_2(measure_t_pred(3)) << ", "<< normalize_2(measure_t_pred(4)) << ", "<< normalize_2(measure_t_pred(5)) << "] " << endl;

    // fix angles differences
    measure_difference(3) = diff_roll;
    measure_difference(4) = diff_pitch;
    measure_difference(5) = diff_yaw;

    // kalman gain
    VectorXd kalman_per_msr_diff = K_t * measure_difference;
    kalman_per_msr_diff(3) = normalize_2(kalman_per_msr_diff(3));
    kalman_per_msr_diff(4) = normalize_2(kalman_per_msr_diff(4));
    kalman_per_msr_diff(5) = normalize_2(kalman_per_msr_diff(5));

    cout << "[measure_difference orientation: " << measure_difference(3) << ", " << measure_difference(4) << ", " << measure_difference(5) << "] " << endl;
    cout << "[K_t * msr. diff. orientation: " << kalman_per_msr_diff(3) << ", " << kalman_per_msr_diff(4) << ", " << kalman_per_msr_diff(5) << "] " << endl;

    // calculate belief
    VectorXd stato_filtrato = stato_t_predetto + kalman_per_msr_diff;
	E_t = (MatrixXd::Identity(12,12) - K_t * H_t) * E_t_pred;

    // normalize angles between -PI and PI because of the sum: stato_t_predetto + (...)
    stato_filtrato(3) = normalize_2(stato_filtrato(3));
    stato_filtrato(4) = normalize_2(stato_filtrato(4));
    stato_filtrato(5) = normalize_2(stato_filtrato(5));

    // update particle values
    particle.setParticleState(stato_filtrato);
	particle.setParticleSigma(E_t);


    //DEBUG:stampare stato_t_predetto
    cout << endl << "[stato_t ] [pose: " << stato_t(0) << ", " <<  stato_t(1) <<  ", " << stato_t(2) << "] [orientation: " << stato_t(3) <<  ", " << stato_t(4) <<  ", " << stato_t(5) << "] "<< endl;
    cout << "[stato_t_predetto ] [pose: " << stato_t_predetto(0) << ", " <<  stato_t_predetto(1) <<  ", " << stato_t_predetto(2) << "] [orientation: " << stato_t_predetto(3) <<  ", " << stato_t_predetto(4) <<  ", " << stato_t_predetto(5) << "] "<< endl;
    cout << "[stato_t_filtrato ] [pose: " << stato_filtrato(0) << ", " <<  stato_filtrato(1) <<  ", " << stato_filtrato(2) << "] [orientation: " << stato_filtrato(3) <<  ", " << stato_filtrato(4) <<  ", " << stato_filtrato(5) << "] "<< endl;
    cout << "--------------------------------------------------------------------------------" << endl;
}



void LayoutManager::componentsEstimation(){
	// STEP 1: SAMPLING (PREDICT COMPONENTS POSES)
	sampling();

	// STEP 2: PERTURBATE COMPONENT POSES
	componentsPerturbation();

	// STEP 3: WEIGHT LAYOUT-COMPONENTS
	vector<Particle>::iterator itr;
	for( itr = current_layout.begin(); itr != current_layout.end(); itr++ ){
        itr->calculateComponentsWeight();  /// andiamo a vedere quanto bene fitta la componente della singola particella nella realtà
	}
}

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
        //TODO: calculate score again, with new detections (if available)

	}


	return current_layout;
}


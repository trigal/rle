/*
 * LayoutManager.h
 *
 *  Created on: May 25, 2014
 *      Author: dario
 */

#ifndef LAYOUTMANAGER_H_
#define LAYOUTMANAGER_H_

#include "VisualOdometry.h"
#include "particle/Particle.h"
#include <vector>
#include "nav_msgs/Odometry.h"
#include <Eigen/Dense>	//used for motion threshold matrix
#include <Eigen/Core>
using namespace Eigen;
using Eigen::MatrixXd;
using std::vector;

/**
 *----------------------------------------------------------------------------------------------
 * SPECIFICHE:
 * - pose particle -> filtro EKF
 * 	 pose componenti -> filtro particelle
 *
 * - score -> confronto tra predict ed eq. di misura
 * 	 weight -> quanto pesa nella somma di tutte le componenti
 *
 *		pose: 12x1 6DoF + 6 speed
 *		covarianza 12x12
 *
 *		controllo se mi sono mosso: norm(pose_t - pose_t-1) < threshold
 *
 *		SCORE = SCORE_MOTION_MODEL(kalman gain) + sum(SCORE_COMPONENTS)
 *
 *	EQUAZIONE DI MISURA:
 *		mi arriva la misura dal tracker (visual odometry), vettore 6 elementi (6DoF pose).
 *		L'equazione di misura sarà una matrice (vedi foglio). La derivata è paro paro
 *
 *		correggere descrizione calcolo score
 *
 *--------------------------------------------------------------------------------------------------------
 *	CALCOLO V_t+1
 *
 *	PREDICTION:
 *		considero sempre l'equazione v t+1 = v t + R
 *
 *	UPDATE:
 *	caso 1: non mi arrivano dati da visual odometry
 *			l'accelerazione è data dalla costante di smorzamento (vedi formula su foglio)
 *	caso 2: nel caso mi arrivino dati, faccio l'update
 *
 *
 *----------------------------------------------------------------------------------------------
 *
 * DA CHIEDERE:
 *
 * - le equazioni di misura (usate x calcolare parte dello score) sono in visual odometry?
 *		da mettere su particle-component
 *
 *----------------------------------------------------------------------------------------------
 */



/**
 * Implementation of the current layout estimator
 */
class LayoutManager {
public:
	//da far tornare private
    void particleEstimation(Particle & particle);
	VisualOdometry visual_odometry;	/// used for getting car motion

private:

    bool is_new_detection;				/// indicates detectors found new detections
    vector<double> score_vector;
    vector<Particle> current_layout;	/// stores the current layout
    VectorXd current_measurement;
    double motion_threshold;


	/**
	 * Sampling from the state transition p(x_t | u_t , x_t-1):
	 * we propagate the particle and its components with the motion model
	 * and generate a predicted particle-set
	 */
	void sampling();

	/**
	 * Resampling sul particle-set predetto, utilizzando lo score delle particelle:
	 * chi ha peso più alto è più probabile che venga preso [roulette-wheel]
	 * @param particle-set predetto
	 * @return particle-set con resampling
	 */
	void resampling();

	void componentsPerturbation();

	bool checkHasMoved();

	void componentsEstimation();

	void calculateScore();

public:

    void setParticlesDelta(double delta);

    VectorXd getCurrentMeasurement(){
        return current_measurement;
    }

    void setCurrentMeasurement(VectorXd & msr){
        current_measurement = msr;
    }

	VisualOdometry getVisualOdometry(){
		return visual_odometry;
	}

	void setVisualOdometry(VisualOdometry& v_odom){
		visual_odometry = v_odom;
	}

	/**
	 * Genera una stima del layout al tempo t a partire dal currentLayout
	 * @return particle set al tempo t
	 */
	vector<Particle> layoutEstimation();


	/**
	 * Returns last estimated layout (used for other detectors' feedback)
	 * @return particle-set
	 */
	vector<Particle> getCurrentLayout(){
		return current_layout;
	}


	/**
	 * Used to set the current layout (usually this will called only one time, during the startup)
	 * @param p_set
	 */
	void setCurrentLayout(vector<Particle>& p_set){
		current_layout = p_set;
	}

	/**
	 * @return current odometry
	 */
	MatrixXd getOdometry(){
		return visual_odometry.getOdometry();
	}

	/**
	 * Sets current odometry
	 * @param odm
	 */
	void setOdometry(MatrixXd& odm){
		visual_odometry.setOdometry(odm);
	}



	// costructor, destructor, copy costr, assignment
	LayoutManager(){
		is_new_detection = false;
		motion_threshold  = 0.05;
	};
//	LayoutManager(bool p1, vector<double>& p2, vector<Particle>& p3, VisualOdometry& p4) :
//		is_new_detection(p1), score_vector(p2), current_layout(p3), visual_odometry(p4) {
//		motion_threshold  = 0.05;
//	};
//	LayoutManager(vector<Particle>& p3, VisualOdometry& p4) :
//			is_new_detection(false), score_vector(0), current_layout(p3), visual_odometry(p4) {
//		motion_threshold  = 0.05;
//	};
	~LayoutManager(){
		score_vector.resize(0);
		//currentLayout.resize(0);
    }
	LayoutManager(const LayoutManager &other);
	LayoutManager& operator=(const LayoutManager&);
};



#endif /* LAYOUTMANAGER_H_ */

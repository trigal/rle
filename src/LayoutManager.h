/*
 * LayoutManager.h
 *
 *  Created on: May 25, 2014
 *      Author: dario
 */

#ifndef LAYOUTMANAGER_H_
#define LAYOUTMANAGER_H_

#include "Odometry.h"
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
                 // - score -> confronto tra predict ed eq. di misura
                 // - weight -> quanto pesa nella somma di tutte le componenti
 *
 *
 * - CHECK HAS MOVED: norm(pose_t - pose_t-1) < threshold
 *
 *  SCORE = SCORE_MOTION_MODEL(kalman gain) + sum(SCORE_COMPONENTS)
 *
 *	EQUAZIONE DI MISURA:
 *		mi arriva la misura dal tracker (visual odometry), vettore 6 elementi (6DoF pose).
 *		L'equazione di misura sarà una matrice (vedi foglio). La derivata è paro paro
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
 * DA FARE:
 * - trovare nuovo motion model
 * - implementare particle filter
 *----------------------------------------------------------------------------------------------
 */



/**
 * Implementation of the current layout estimator
 */
class LayoutManager {
public:
	//da far tornare private
    void particleEstimation(Particle & particle);
    Odometry visual_odometry;	/// used for getting car motion
    static double delta_t;

private:
    bool new_detections;				/// indicates detectors found new detections
    vector<double> score_vector;
    vector<Particle> current_layout;	/// stores the current layout
    double motion_threshold;
    VectorXd msr_state;
    MatrixXd msr_cov;


    bool checkHasMoved();

    /**
     * STEP 1: SAMPLING (PREDICT COMPONENTS POSES)
     * STEP 2: PERTURBATE COMPONENT POSES
     * STEP 3: WEIGHT LAYOUT-COMPONENTS
     */
    void componentsEstimation();

    /**
	 * Sampling from the state transition p(x_t | u_t , x_t-1):
	 * we propagate the particle and its components with the motion model
     * and genera//currentLayout.resize(0);te a predicted particle-set
	 */
	void sampling();

    void componentsPerturbation();

    void calculateLayoutComponentsWeight();

	/**
	 * Resampling sul particle-set predetto, utilizzando lo score delle particelle:
	 * chi ha peso più alto è più probabile che venga preso [roulette-wheel]
	 * @param particle-set predetto
	 * @return particle-set con resampling
	 */
	void resampling();


    /**
     * FORMULA CALCOLO SCORE
     *
     * Cardinalità unaria (Particella presa INDIVIDUALMENTE)
     *  1- Kalman gain sulla pose della particella
     *  2- Somma dei WEIGHT delle varie componenti della particella (ottenuti dal filtraggio a particelle)
     *
     * Cardinalità >= 2
     *  1- plausibilità di esistenza tra le varie componenti di stesso tipo (due building sovrapposti ecc.)
     *  2- plausibilità di esistenza tra componenti di diverso tipo (building sovrapposto a una macchina ecc.)
     *
     * Nessuna particella verrà eliminata durante il procedimento di calcolo dello score,
     * essa sarà mantenuta in vita nonostante lo score sia basso.
     * In questo modo si evita la possibilità di eliminare dal particle-set ipotesi plausibili che abbiano ricevuto
     * uno score di valore basso per motivi di natura diversa.
     *
     */
	void calculateScore();

public:

    void setMeasureState(VectorXd& msrstate){
        msr_state = msrstate;
    }

    VectorXd getMeasureState(){
        return msr_state;
    }

    void setMeasureCov(MatrixXd& msrcov){
        msr_cov = msrcov;
    }

    MatrixXd getMeasureCov(){
        return msr_cov;
    }

    Odometry getVisualOdometry(){
		return visual_odometry;
	}

    void setVisualOdometry(Odometry& v_odom){
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

    // costructor
	LayoutManager(){
		new_detections = false;
		motion_threshold  = 0.05;
    }

    // destructor
	~LayoutManager(){
		score_vector.resize(0);
        current_layout.resize(0);
        msr_state.resize(0);
        msr_cov.resize(0,0);
    }
	LayoutManager(const LayoutManager &other);
	LayoutManager& operator=(const LayoutManager&);
};

#endif /* LAYOUTMANAGER_H_ */

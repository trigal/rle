/*
 * ParticleComponent.h
 *
 *  Created on: Jun 2, 2014
 *      Author: dario
 */

#ifndef PARTICLECOMPONENT_H_
#define PARTICLECOMPONENT_H_

#include <vector>
#include <Eigen/Dense>	//used for component pose matrix
using Eigen::VectorXd;
using std::vector;

class ParticleComponent {
private:
	VectorXd componentPose;	 /// current particle-component pose (12x12: 6DoF + 6 speed derivates)

public:
	/**
	 * This function compute particle-componenet score, used during P.F. resampling
	 *
	 * Associa uno score ad ogni particella del particle set predetto
	 * Ogni score è memorizzato in score_vector
	 * @param particle set predetto
	 * @param misura al tempo t
	 * @param altri valori dati dai detector
	 * @return vettore pesi associato al particle set predetto
	 *
	 */
	void calculateScore();

	/**
	 * @return particle-component pose
	 */
	VectorXd getComponentPose(){
		return this->componentPose;
	};

	/**
	 * Set this particle-component pose
	 * @param pose
	 */
	void setComponentPose(VectorXd& pose){
		this->componentPose = pose;
	}

	//constructor
	ParticleComponent();
	ParticleComponent(VectorXd& pose) : componentPose(pose) {};


	//destructor
	virtual ~ParticleComponent(){
		componentPose.resize(0);
	}
};

#endif /* PARTICLECOMPONENT_H_ */

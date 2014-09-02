/*
 * Particle.h
 *
 *  Created on: Jun 2, 2014
 *      Author: dario
 */

#ifndef PARTICLE_H_
#define PARTICLE_H_

#include "ParticleComponent.h"
#include "MotionModel.h"
#include <vector>
#include <Eigen/Dense>	//used for pose matrix
#include <Eigen/Core>
using namespace Eigen;
using std::vector;

class Particle {

private:
	unsigned int id;			/// particle id
	VectorXd particle_state;	/// particle state (12x1: 6DoF pose + 6 Speed Derivates)
	MatrixXd particle_sigma;	/// particle state error covariance (12x12)
	vector<ParticleComponent*> particle_components; /// array of particle-components

public:
	MotionModel mtn_model;	/// particle motion model

	/**
	 * This function will use "motion-model" class to propagate all particle's components
	 */
	void propagateParticleComponents();

	/**
	 * This function will use "motion-model" class to propagate particle pose
	 */
	VectorXd propagateParticlePose();

	/**
	 * This function will calculate all the components' score
	 */
	void calculateComponentsScore();

	/**
	 * Add new component to the particle
	 * @param component
	 */
	void addComponent(ParticleComponent* component){
		this->particle_components.push_back(component);
	}

	/**
	 * Remove the selected component from the particle
	 * @param component
	 */
	void removeComponent(ParticleComponent* component){
//		this->particleComponents.erase(component);
//
//
//		#include <algorithm>
//		std::vector<int>::iterator position = std::find(vector.begin(), vector.end(), 8);
//		if (position != vector.end()) // == vector.end() means the element was not found
//			myVector.erase(position);
	}


	//getters&setters -------------------------------------------------------------
	int getId() const {
		return id;
	}

	void setId(int id) {
		this->id = id;
	}

	vector<ParticleComponent*> getParticleComponents(){
		return particle_components;
	}

	VectorXd getParticleState(){
		return particle_state;
	}

	void setParticleState(const VectorXd& p_state){
		particle_state = p_state;
	}

	MatrixXd getParticleSigma(){
		return particle_sigma;
	}

	void setParticleSigma(MatrixXd& p_sigma){
		particle_sigma = p_sigma;
	}

	//constructor -------------------------------------------------------------
	Particle(){
		this->id = 0;
		this->particle_state = VectorXd::Zero(12);
		this->particle_sigma = MatrixXd::Zero(12,12);
	};
	Particle(unsigned int num, MotionModel& mt_md) : id(num), mtn_model(mt_md) {};
	Particle(unsigned int num, VectorXd& state, MotionModel& mt_md )
		: id(num), particle_state(state), mtn_model(mt_md)  {};
	Particle(unsigned int num, VectorXd& state, MatrixXd& state_sigma, MotionModel& mt_md)
		: id(num), particle_state(state), particle_sigma(state_sigma), mtn_model(mt_md) {};

	//destructor
	virtual ~Particle() {
//		id=0;
//		vector<ParticleComponent*>::iterator itr;
//		for (itr = particleComponents.begin(); itr != particleComponents.end(); itr++){
//			ParticleComponent* comp = *itr;
//			delete comp;
//		}
	}

	//copy constructor

	//assignment


};

#endif /* PARTICLE_H_ */

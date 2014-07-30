/*
 * Particle.cpp
 *
 *  Created on: Jun 2, 2014
 *      Author: dario
 */

#include "ParticleComponent.h"
#include "Particle.h"
#include "MotionModel.h"
#include <vector>
#include <Eigen/Dense>	//used for pose matrix
#include <Eigen/Core>
using namespace Eigen;
using std::vector;

void Particle::propagateParticleComponents(){
	vector<ParticleComponent*>::iterator itr;
	for (itr = particle_components.begin(); itr != particle_components.end(); itr++){
		ParticleComponent* p_comp = *itr;

		mtn_model.propagateComponent(p_comp);
	}
}

void Particle::calculateComponentsScore(){
	vector<ParticleComponent*>::iterator itr;
	for (itr = particle_components.begin(); itr != particle_components.end(); itr++){
		ParticleComponent* p_comp = *itr;

		p_comp->calculateScore();
	}
}

VectorXd Particle::propagateParticlePose(){
	return mtn_model.propagatePose(particle_state);
}


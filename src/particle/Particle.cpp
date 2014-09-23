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


/**
 * Update every particle's components pose using motion model equations
 */
void Particle::propagateParticleComponents(){
    vector<ParticleComponent>::iterator itr;
    for (itr = particle_components.begin(); itr != particle_components.end(); itr++){

        // propagate component pose with motion model equations
        VectorXd new_pose = mtn_model.propagateComponent(*itr);

        // update pose with predicted one
        itr->setComponentPose(new_pose);
    }
}

void Particle::calculateComponentsWeight(){
//	vector<ParticleComponent*>::iterator itr;
//	for (itr = particle_components.begin(); itr != particle_components.end(); itr++){
//		ParticleComponent* p_comp = *itr;

//		p_comp->calculateScore();
//	}
}


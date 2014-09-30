/*
 * Particle.cpp
 *
 *  Created on: Jun 2, 2014
 *      Author: dario
 */

#include "LayoutComponent.h"
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
void Particle::propagateLayoutComponents(){
    vector<LayoutComponent*>::iterator itr;
    for (itr = particle_components.begin(); itr != particle_components.end(); itr++){

        // propagate component pose with motion model equations
        VectorXd pc_state = (*itr)->getComponentState();
        VectorXd new_pose = mtn_model.propagateComponent(pc_state);

        // update pose with predicted one
        (*itr)->setComponentState(new_pose);
    }
}



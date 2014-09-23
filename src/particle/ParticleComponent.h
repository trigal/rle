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
    unsigned int particle_id;
    unsigned int component_id;
    VectorXd component_state;	 /// current particle-component pose (12x1: 6DoF + 6 speed derivates)

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


    // GETTERS & SETTERS ---------------------------------------------------------------------------------
    unsigned int getComponentId(){return component_id;}
    void setComponentId(unsigned int id){ component_id = id; }
    unsigned int getParticleId(){return particle_id;}
    void setParticletId(unsigned int id){ particle_id = id; }

	/**
	 * @return particle-component pose
	 */
	VectorXd getComponentPose(){
        return this->component_state;
    }

	/**
	 * Set this particle-component pose
	 * @param pose
	 */
	void setComponentPose(VectorXd& pose){
        this->component_state = pose;
	}
    // ---------------------------------------------------------------------------------------------------



	//constructor
	ParticleComponent();
    ParticleComponent(VectorXd& pose) : component_id(0), component_state(pose) {}
    ParticleComponent(unsigned int id, VectorXd& pose) : component_id(id), component_state(pose) {}
    ParticleComponent(unsigned int p_id, unsigned int id, VectorXd& pose) :
        particle_id(p_id), component_id(id), component_state(pose) {}

    //destructor
	virtual ~ParticleComponent(){
        component_state.resize(0);
	}
};

#endif /* PARTICLECOMPONENT_H_ */

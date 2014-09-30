    /*
     * Particle.h
     *
     *  Created on: Jun 2, 2014
     *      Author: dario
     */

    #ifndef PARTICLE_H_
    #define PARTICLE_H_

    #include "LayoutComponent.h"
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
        vector<LayoutComponent*> particle_components; /// array of particle-components
        MatrixXd kalman_gain;     /// kalman gain got while estimating the pose
        double particle_score;   /// score got with particle-score formula

    public:
        MotionModel mtn_model;	/// particle motion model

        /**
         * This function will use "motion-model" class to propagate all particle's components
         */
        void propagateLayoutComponents();

        /**
         * Add new component to the particle
         * @param component
         */
        void addComponent(LayoutComponent* component){
            this->particle_components.push_back(component);
        }

        /**
         * Remove the selected component from the particle
         * @param component
         */
    //    void removeComponent(LayoutComponent component){
    //		this->LayoutComponents.erase(component);
    //
    //
    //		#include <algorithm>
    //		std::vector<int>::iterator position = std::find(vector.begin(), vector.end(), 8);
    //		if (position != vector.end()) // == vector.end() means the element was not found
    //			myVector.erase(position);
    //	}


        //getters & setters -----------------------------------------------------------------
        int getId() const { return id; }
        void setId(int id) { id = id; }

        vector<LayoutComponent*> getLayoutComponents(){ return particle_components;}
        void setLayoutComponents(vector<LayoutComponent*> vec){ particle_components = vec; }

        VectorXd getParticleState(){ return particle_state; }
        void setParticleState(const VectorXd& p_state){ particle_state = p_state;}

        MatrixXd getParticleSigma(){ return particle_sigma; }
        void setParticleSigma(MatrixXd& p_sigma){ particle_sigma = p_sigma; }

        MatrixXd getKalmanGain(){return kalman_gain;}
        void setKalmanGain(MatrixXd& k){kalman_gain=k;}

        double getParticleScore(){return particle_score;}
        void setParticleScore(double score){particle_score = score;}

        //constructor ----------------------------------------------------------------------
        Particle(){
            id = 0;
            kalman_gain = MatrixXd::Zero(12,12);
            particle_state = VectorXd::Zero(12);
            particle_sigma = MatrixXd::Zero(12,12);
        }
        Particle(unsigned int num, MotionModel& mt_md) : id(num), mtn_model(mt_md) {}
        Particle(unsigned int num, VectorXd& state, MotionModel& mt_md )
            : id(num), particle_state(state), mtn_model(mt_md)  {}
        Particle(unsigned int num, VectorXd& state, MatrixXd& state_sigma, MotionModel& mt_md)
            : id(num), particle_state(state), particle_sigma(state_sigma), mtn_model(mt_md) {}

        //destructor -------------------------------------------------------------
        ~Particle(){
            id = 0;
            kalman_gain.resize(0,0);
            particle_state.resize(0);
            particle_sigma.resize(0,0);
            particle_components.resize(0);
        }
    };

    #endif /* PARTICLE_H_ */

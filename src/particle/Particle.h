/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Author:    Dario Limongi                                              *
 *   Email:     dario.limongi@gmail.com                                    *
 *   Date:      02/06/2014                                                 *
 *                                                                         *
 ***************************************************************************/

#ifndef PARTICLE_H_
#define PARTICLE_H_

#include <boost/ptr_container/ptr_vector.hpp>

#include "LayoutComponent_RoadState.h"
#include "LayoutComponent.h"
#include "MotionModel.h"
#include "../MeasurementModel.h"
#include "../Utils.h"

#include <Eigen/Dense>  //used for pose matrix
#include <Eigen/Core>

#include <vector>
#include <typeinfo>

using namespace Eigen;
using std::vector;


class Particle
{

private:
    unsigned int particle_id;   /// particle id
    State6DOF particle_state;   /// particle state (12x1: 6DoF pose + 6 Speed Derivates)
    MatrixXd particle_sigma;    /// particle state error covariance (12x12)
    vector<LayoutComponent*> particle_components; /// array of particle-components

    MatrixXd kalman_gain;    /// kalman gain got while estimating the pose
    double particle_score;   /// score got with particle-score formula
    MotionModel particle_mtn_model;  /// particle motion model


public:

    int in_cluster;
    double distance_to_closest_segment;

    double pose_diff_score_component;
    double final_angle_diff_score_component;

    /**
     * This function will use "motion-model" class to propagate all particle's components
     */
    void propagateLayoutComponents();

    /**
     * Add new component to the particle
     * @param component
     */
    void addComponent(LayoutComponent* component)
    {
        if (dynamic_cast<LayoutComponent_RoadState* >(component))
        {
            ROS_DEBUG_STREAM("Adding Component roadStateComponent with componentID: " << component->getComponentId());
            this->particle_components.push_back(component);
        }
    }

    /**
     * Remove the selected component from the particle
     * @param component
     */
//    void removeComponent(LayoutComponent component){
//      this->LayoutComponents.erase(component);
//
//
//      #include <algorithm>
//      std::vector<int>::iterator position = std::find(vector.begin(), vector.end(), 8);
//      if (position != vector.end()) // == vector.end() means the element was not found
//          myVector.erase(position);
//  }


    //getters & setters -----------------------------------------------------------------
    int getId() const
    {
        return particle_id;
    }
    void setId(int id)
    {
        particle_id = id;
    }

    void particlePoseEstimation(MeasurementModel *odometry, double deltaTimerTime = 0.0f, double deltaOdomTime = 0.0f);

    vector<LayoutComponent*> getLayoutComponents()
    {
        return particle_components;
    }
    vector<LayoutComponent*>* getLayoutComponentsPtr()
    {
        return &particle_components;
    }
    void setLayoutComponents(vector<LayoutComponent*> vec)
    {
        particle_components = vec;
    }

    State6DOF getParticleState()
    {
        return particle_state;
    }
    void setParticleState(const State6DOF& p_state)
    {
        particle_state = p_state;
    }
    void setParticleVelocities(const State6DOF& p_state)
    {
        particle_state._rotational_velocity    = p_state._rotational_velocity;
        particle_state._translational_velocity = p_state._translational_velocity;
    }

    MatrixXd getParticleSigma()
    {
        return particle_sigma;
    }
    void setParticleSigma(MatrixXd& p_sigma)
    {
        particle_sigma = p_sigma;
    }

    MatrixXd getKalmanGain()
    {
        return kalman_gain;
    }
    void setKalmanGain(MatrixXd& k)
    {
        kalman_gain = k;
    }

    double getParticleScore()
    {
        return particle_score;
    }
    void setParticleScore(double score)
    {
        particle_score = score;
    }

    MotionModel getMotionModel()
    {
        return particle_mtn_model;
    }
    MotionModel* getMotionModelPtr()
    {
        return &particle_mtn_model;
    }
    void setMotionModel(MotionModel m)
    {
        particle_mtn_model = m;
    }
    void setMotionModelErrorCovariance(double position_uncertainty,
                                       double orientation_uncertainty,
                                       double linear_uncertainty,
                                       double angular_uncertainty)
    {
        particle_mtn_model.setErrorCovariance(
            position_uncertainty,
            orientation_uncertainty,
            linear_uncertainty,
            angular_uncertainty
        );
    }

    //constructor ----------------------------------------------------------------------
    Particle()
    {
        particle_id = 0;
        kalman_gain = MatrixXd::Zero(12, 12);
        particle_sigma = MatrixXd::Zero(12, 12);
        particle_score = 0;
    }
    Particle(unsigned int id, MotionModel mt_md) : particle_id(id), particle_mtn_model(mt_md)
    {
        kalman_gain = MatrixXd::Zero(12, 12);
        particle_sigma = MatrixXd::Zero(12, 12);
        particle_score = 0;
    }
    Particle(unsigned int id, State6DOF& state, MotionModel mt_md )
        : particle_id(id), particle_state(state), particle_mtn_model(mt_md)
    {
        particle_sigma = MatrixXd::Zero(12, 12);
        particle_score = 0;
    }
    Particle(unsigned int id, State6DOF state, MatrixXd state_sigma, MotionModel mt_md)
        : particle_id(id), particle_state(state), particle_sigma(state_sigma), particle_mtn_model(mt_md)
    {
        ROS_DEBUG_STREAM("Particle new_particle(particle_id, p_pose, p_sigma, default_mtn_model)");
        ROS_DEBUG_STREAM("Motion Model Absolute Translational Velocity errors:\t" << mt_md.propagate_translational_absolute_vel_error_x << "\t" << mt_md.propagate_translational_absolute_vel_error_y << "\t" << mt_md.propagate_translational_absolute_vel_error_z);
        ROS_DEBUG_STREAM("Motion Model    %     Translational Velocity errors:\t" << mt_md.getPropagate_translational_percentage_vel_error_x() << "\t" << mt_md.getPropagate_translational_percentage_vel_error_y() << "\t" << mt_md.getPropagate_translational_percentage_vel_error_z());
        kalman_gain = MatrixXd::Zero(12, 12);
        particle_score = 0;
    }

    //destructor -------------------------------------------------------------
    ~Particle()
    {
        particle_id = 0;
        kalman_gain.resize(0, 0);
        particle_sigma.resize(0, 0);
        particle_components.resize(0);
        particle_score = 0;

        // delete all particle's components
        for (int i = 0; i < particle_components.size(); ++i)
        {
            delete particle_components.at(i);
        }
    }
};

#endif /* PARTICLE_H_ */

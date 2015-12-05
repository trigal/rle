#ifndef LAYOUTCOMPONENT_H
#define LAYOUTCOMPONENT_H

// Forward declaration, refs #523
class Particle; //also in Particle.h https://en.wikipedia.org/wiki/Circular_dependency

#include <vector>
#include <Eigen/Eigen>
#include <ros/ros.h>


using namespace Eigen;
using std::vector;


class LayoutComponent
{

protected:
    Particle *particle;         /// Pointer to the parent particle. refs #523
    unsigned int particle_id;   /// Tells particle ID of where this component is living. This is the id of the *particle
    unsigned int component_id;  /// Component ID
    double component_weight;    /// Used for resampling
    VectorXd component_state;   /// Particle-component pose
    MatrixXd component_cov;     /// Particle-component uncertainty


public:

    ///
    /// \brief calculateComponentScore
    /// This function compute particle-componenet weight, used during P.F. resampling
    ///
    virtual void calculateComponentScore() = 0;

    ///
    /// \brief componentPoseEstimation
    /// \todo Rename in componentStateEstimation
    ///
    /// This function is the implementation of pose propagation of layout component
    /// Can be a POSE to be moved or a STATE like RoadState
    ///
    virtual void componentPoseEstimation() = 0;

    ///
    /// \brief componentPerturbation
    ///
    virtual void componentPerturbation() = 0;


    // GETTERS & SETTERS ---------------------------------------------------------------------------------
    unsigned int getComponentId()
    {
        return component_id;
    }
    void setComponentId(unsigned int id)
    {
        component_id = id;
    }

    unsigned int getParticleId()
    {
        return particle_id;
    }
    void setParticleId(unsigned int id)
    {
        particle_id = id;
    }

    double getComponentWeight()
    {
        return component_weight;
    }
    void setComponentWeight(double w)
    {
        component_weight = w;
    }

    VectorXd getComponentState()
    {
        return component_state;
    }
    void setComponentState(VectorXd pose)
    {
        ROS_DEBUG_STREAM(">>>setComponentState called");
        component_state = pose;
        ROS_DEBUG_STREAM("<<<setComponentState called");
    }

    MatrixXd getComponentCov()
    {
        return component_cov;
    }
    void setComponentCov(MatrixXd cov)
    {
        component_cov = cov;
    }
    // ---------------------------------------------------------------------------------------------------

    // destructor
    ~LayoutComponent()
    {
        particle = NULL; //refs #523
        particle_id = 0;
        component_id = 0;
        component_weight = 0;
        component_state.resize(0);
        component_cov.resize(0, 0);
    }

    // constructor
    LayoutComponent()
    {
        ROS_DEBUG_STREAM(__PRETTY_FUNCTION__);
        particle = NULL; //refs #523
        particle_id = 0;
        component_id = 0;
        component_weight = 0;
        component_state = VectorXd::Zero(12);
        component_cov = MatrixXd::Zero(12, 12);
    }
//    LayoutComponent(VectorXd& pose) : component_id(0), component_state(pose) {}
//    LayoutComponent(unsigned int id, VectorXd& pose) : component_id(id), component_state(pose) {}
//    LayoutComponent(unsigned int p_id, unsigned int id, VectorXd& pose) :
//        particle_id(p_id), component_id(id), component_state(pose) {}

    Particle *getParticle() const;      ///refs #523
    void setParticle(Particle *value);  ///refs #523
};


#endif // LAYOUTCOMPONENT_H

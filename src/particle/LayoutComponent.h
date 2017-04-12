#ifndef LAYOUTCOMPONENT_H
#define LAYOUTCOMPONENT_H

// Forward declaration, refs #523
class Particle; //also in Particle.h https://en.wikipedia.org/wiki/Circular_dependency

#include <vector>
#include <Eigen/Eigen>
#include <ros/ros.h>

using namespace std;
using namespace Eigen;
using std::vector;


class LayoutComponent
{

protected:
    //ROS_DEPRECATED Particle *particle;         /// Pointer to the parent particle. refs #523
    shared_ptr<Particle> particlePtr;          /// shared Pointer to the parent particle. refs #523
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
    virtual void componentPoseEstimation(int i) = 0;

    ///
    /// \brief componentPerturbation
    ///
    virtual void componentPerturbation() = 0;

    /**
     * @brief getAlphas this function returns the sum of the alphas used in the
     * calculateComponentScore. Before #534 these values were stored inside the
     * LayoutComponent and there was a sumOfAlphas to summarize all the alphas.
     * Replacing that behavior with a for through the Layouts seems to be a
     * better and foward-thinking
     */
    virtual double getAlphas() = 0;

    virtual LayoutComponent* clone() = 0;

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

    /**
     * @brief getComponentWeight the weight is the overall measure of a
     * LayoutComponent. It involves some probability density function and some
     * ALPHA value (maybe more than one). The alphas are summarized by the
     * getAlphas() routine. This weight is used to calculate the overall SCORE
     * of the particle (what has more than one component and so more weights)..
     *
     * this changes are related to #535 / #534
     *
     * @return the current component weight.
     */
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
        //particle = NULL; //refs #523 TODO: delete thin line when exiting pointersVersion branch
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
        //particle = NULL; //refs #523 TODO: delete thin line when exiting pointersVersion branch
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

    //ROS_DEPRECATED Particle *getParticle() const;      ///refs #523
    //ROS_DEPRECATED void setParticle(Particle *value);  ///refs #523
    shared_ptr<Particle> getParticlePtr() const;
    void setParticlePtr(const shared_ptr<Particle> &value);
};


#endif // LAYOUTCOMPONENT_H

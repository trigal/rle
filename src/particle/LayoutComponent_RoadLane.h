#ifndef LAYOUTCOMPONENT_ROADLANE_H
#define LAYOUTCOMPONENT_ROADLANE_H

#include "LayoutComponent.h"
#include <iostream>

using namespace std;

class LayoutComponent_RoadLane : public LayoutComponent
{
public:

    /**
     * Implementation of pure virtual method 'calculateWeight'
     */
    void calculateWeight(){
        cout << "Calculating weight of ROAD LANE component ID: " << component_id << " that belongs to particle ID: " <<particle_id << endl;
    }

    /**
     * Implementation of pure virtual method 'componentPerturbation'
     */
    void componentPerturbation(){
        cout << "Perturbating ROAD LANE component ID: " << component_id << " that belongs to particle ID: " <<particle_id << endl;
    }

    /**
     * @brief LayoutComponent_RoadLane constructor
     * @param p_id
     * @param c_id
     * @param c_state
     * @param c_cov
     */
    LayoutComponent_RoadLane(const unsigned int p_id, const unsigned int c_id, const VectorXd& c_state, const MatrixXd& c_cov){
        particle_id = p_id;
        component_id = c_id;
        weight = 0;
        component_state = c_state;
        component_cov = c_cov;
    }

    LayoutComponent_RoadLane(){
        particle_id = 0;
        component_id = 0;
        weight = 0;
        component_state = VectorXd::Zero(12);
        component_cov = MatrixXd::Zero(12,12);
    }

    ~LayoutComponent_RoadLane(){
        particle_id = 0;
        component_id = 0;
        weight = 0;
        component_state.resize(0);
        component_cov.resize(0,0);
    }
};

#endif // LAYOUTCOMPONENT_ROADLANE_H

#ifndef LAYOUTCOMPONENT_CROSSING_H
#define LAYOUTCOMPONENT_CROSSING_H

#include "LayoutComponent.h"
#include <iostream>

using namespace std;

class LayoutComponent_Crossing : public LayoutComponent
{
private:

    struct road
    {
        float width;
        double rotation; // rotation from the incoming road (where the observer is)
    };

    float center_x;
    float center_y;
    int num_ways;  //number of roads approaching the intersection
    vector<road> intersection_roads; //list of all roads approaching the intersection, including the incoming one




public:

    /**
     * Implementation of pure virtual method 'calculateWeight'
     */
    void calculateComponentScore()
    {
        cout << "Calculating weight of CROSSING component ID: " << component_id << " that belongs to particle ID: " << particle_id << endl;
    }

    /**
     * Implementation of pure virtual method 'componentPerturbation'
     */
    void componentPerturbation(){
        //  we can slightly change the parameters, like moving the center, change width / orientation of one or more roads,
        //  or even add / remove a road
        cout << "Perturbating CROSSING component ID: " << component_id << " that belongs to particle ID: " << particle_id << endl;
    }

    /**
     * Implementation of pure virtual method 'componentPoseEstimation'
     */
    void componentPoseEstimation()
    {
        //Qua si potrebbe spostare il centro dell'incrocio vicino all'osservatore in base all'odometria / velocità

        cout << "Propagating and estimating CROSSING component pose. ID: " << component_id << " that belongs to particle ID: " << particle_id << endl;
    }


    // Getters and setters ----------------------------------------------------------------------
    float        getCenter_x()               const;
    float        getCenter_y()               const;
    int          getNum_ways()               const;
    void         setCenter_x                 (float  value);
    void         setCenter_y                 (float value);
    //void          setNum_ways                 (int value);

    void addRoad(float width, double rotation);

    /**
     * @brief LayoutComponent_Crossing constructor
     * @param p_id
     * @param c_id
     * @param c_state
     * @param c_cov
     */
    LayoutComponent_Crossing(const unsigned int p_id, const unsigned int c_id, float center_x, float center_y, const VectorXd& c_state, const MatrixXd& c_cov)
    {
        particle_id = p_id;
        component_id = c_id;
        component_weight = 0;
        component_state = c_state;
        component_cov = c_cov;

        this->center_x = center_x;
        this->center_y = center_y;
        //this->num_ways = num_ways;
        this->num_ways = 1;

        road incoming_road;
        incoming_road.width = 6;
        incoming_road.rotation = 0;

        intersection_roads.push_back(incoming_road);

    }

    LayoutComponent_Crossing()
    {
        particle_id = 0;
        component_id = 0;
        component_weight = 0;
        component_state = VectorXd::Zero(12);
        component_cov = MatrixXd::Zero(12, 12);
    }

    ~LayoutComponent_Crossing()
    {
        particle_id = 0;
        component_id = 0;
        component_weight = 0;
        component_state.resize(0);
        component_cov.resize(0, 0);

        this->center_x = 0;
        this->center_y = 0;
        this->num_ways = 0;
        intersection_roads.clear();

    }
};

#endif // LAYOUTCOMPONENT_CROSSING_H

#ifndef LAYOUTCOMPONENT_ROADLANE_H
#define LAYOUTCOMPONENT_ROADLANE_H

#include "LayoutComponent.h"
#include <iostream>
#include <ros/ros.h>

using namespace std;

class LayoutComponent_RoadLane : public LayoutComponent
{
private:

    /**
     * PARABOLA EQUATION:
     * x = K1 * (-y + homography_y_resolution)^2 + K2 * (-y + homography_y_resolution) + K3
     */
    double k1; /// First parameter of parabola equation
    double k3; /// Third parameter of parabola equation
    double homography_y_resolution;
    ros::Time timestamp;    /// When parabola was sent by detector
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

    // Getters and setters ----------------------------------------------------------------------
    void setK1(double val){ k1 = val; }
    double getK1() { return k1; }
    void setK3(double val){ k3 = val; }
    double getK3() { return k3; }
    void setTimestamp(ros::Time time) { timestamp = time; }
    ros::Time getTimestamp() { return timestamp; }
    double getHomographyYResolution() { return homography_y_resolution; }
    void setHomographyYResolution(double y_res) { homography_y_resolution = y_res; }

    // Constructors and destructors -------------------------------------------------------------
    LayoutComponent_RoadLane(){
        particle_id = 0;
        component_id = 0;
        weight = 0;
        component_state = VectorXd::Zero(12);
        component_cov = MatrixXd::Zero(12,12);
        k1 = 0;
        k3 = 0;
        timestamp = ros::Time(0);
        homography_y_resolution = 0;
    }
    LayoutComponent_RoadLane(const unsigned int p_id, const unsigned int c_id, double k1, double k3, double y_res, ros::Time timestamp){
        particle_id = p_id;
        component_id = c_id;
        weight = 0;
        component_state = VectorXd::Zero(12);
        component_cov = MatrixXd::Zero(12,12);
        this->k1= k1;
        this->k3 = k3;
        this->homography_y_resolution = y_res;
        this->timestamp = timestamp;
    }
    LayoutComponent_RoadLane(const unsigned int p_id, const unsigned int c_id, const VectorXd& c_state, const MatrixXd& c_cov){
        particle_id = p_id;
        component_id = c_id;
        weight = 0;
        component_state = c_state;
        component_cov = c_cov;
        k1 = 0;
        k3 = 0;
        homography_y_resolution = 0;
        timestamp = ros::Time(0);
    }
    ~LayoutComponent_RoadLane(){
        particle_id = 0;
        component_id = 0;
        weight = 0;
        component_state.resize(0);
        component_cov.resize(0,0);
        k1 = 0;
        k3 = 0;
        homography_y_resolution = 0;
        timestamp = ros::Time(0);
    }
};

#endif // LAYOUTCOMPONENT_ROADLANE_H

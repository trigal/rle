#ifndef LAYOUTCOMPONENT_CROSSING_H
#define LAYOUTCOMPONENT_CROSSING_H

#include "LayoutComponent.h"
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <mutex>

#include "ira_open_street_map/get_closest_crossingXY.h"

using namespace std;
using namespace cv;

class LayoutComponent_Crossing : public LayoutComponent
{
private:

    struct road
    {
        float width;
        double rotation; // rotation from the incoming road (where the observer is)
    };

    int num_ways;  //number of roads approaching the intersection
    vector<road> intersection_roads; //list of all roads approaching the intersection, including the incoming one

    double gridCellSize = 0.1f;
    double min_y = -15.0f;
    double max_y = 15.0f;
    double max_x = 50.0f;

    ros::ServiceClient get_closest_crossingXY_client;

    Point rotatePoint(Point p, double angle);


public:

    //Mat occupancyMap;
    Mat occupancyMap2;
    float center_x;
    float center_y;
    double global_x;
    double global_y;

    Mat sensorOG;
    static bool flag;

    void addRoad(float width, double rotation);
    void computeOccupancyGrid();
    void calculateDistanceCenter(double x, double y);
    void updateSensorOG();
    void setCrossingState(ira_open_street_map::get_closest_crossingXY crossing);


    /**
     * Implementation of pure virtual method 'calculateWeight'
     */
    void calculateComponentScore();

    /**
     * Implementation of pure virtual method 'componentPerturbation'
     */
    void componentPerturbation()
    {
        //  we can slightly change the parameters, like moving the center, change width / orientation of one or more roads,
        //  or even add / remove a road
        cout << "Perturbating CROSSING component ID: " << component_id << " that belongs to particle ID: " << particle_id << endl;
    }

    /**
     * Implementation of pure virtual method 'componentPoseEstimation'
     */
    void componentPoseEstimation();

    double getAlphas()
    {
        return 0.0;
    }


    // Getters and setters ----------------------------------------------------------------------
    float        getCenter_x()               const;
    float        getCenter_y()               const;
    int          getNum_ways()               const;
    void         setCenter_x                 (float  value);
    void         setCenter_y                 (float value);
    //void          setNum_ways                 (int value);

    /**
     * @brief LayoutComponent_Crossing constructor
     * @param p_id
     * @param c_id
     * @param c_state
     * @param c_cov
     */

    LayoutComponent_Crossing(const unsigned int p_id, const unsigned int c_id, float center_x, float center_y,
                             double global_x, double global_y, double width, ros::ServiceClient get_closest_crossingXY_client
                             /*, const VectorXd& c_state, const MatrixXd& c_cov*/)
    {

        this->center_x = center_x;
        this->center_y = center_y;
        this->num_ways = 0;
        this -> global_x = global_x;
        this -> global_y = global_y;

        component_state = VectorXd::Zero(12);
        component_cov = MatrixXd::Zero(12, 12);

        this -> get_closest_crossingXY_client = get_closest_crossingXY_client;

        /*road incoming_road;
        incoming_road.width = width;
        incoming_road.rotation = M_PI;*/

        //intersection_roads.push_back(incoming_road);
        //this -> num_ways ++;

        int n_col = max_x / gridCellSize;
        int n_row = (max_y - min_y) / gridCellSize;
        //occupancyMap = Mat::zeros(n_col, n_row, CV_8UC1);

        occupancyMap2 = Mat::zeros(n_col, n_row, CV_32FC3);
        sensorOG = Mat::zeros(n_col, n_row, CV_32FC3);


    }

    LayoutComponent_Crossing(const unsigned int p_id, const unsigned int c_id, double global_x, double global_y/*, const VectorXd& c_state, const MatrixXd& c_cov*/)
    {

        particle_id = p_id;
        component_id = c_id;
        component_weight = 0;
        component_state = VectorXd::Zero(12);
        component_cov = MatrixXd::Zero(12, 12);
        //component_state = c_state;
        //component_cov = c_cov;

        this->global_x = global_x;
        this->global_y = global_y;
        //this->num_ways = num_ways;
        int n_col = max_x / gridCellSize;
        int n_row = (max_y - min_y) / gridCellSize;
        //occupancyMap = Mat::zeros(n_col, n_row, CV_8UC1);
        occupancyMap2 = Mat::zeros(n_col, n_row, CV_32FC3);

    }

    LayoutComponent_Crossing()
    {

        particle_id = 0;
        component_id = 0;
        component_weight = 0;
        component_state = VectorXd::Zero(12);
        component_cov = MatrixXd::Zero(12, 12);
        int n_col = max_x / gridCellSize;
        int n_row = (max_y - min_y) / gridCellSize;
        //occupancyMap = Mat::zeros(n_col, n_row, CV_8UC1);
        occupancyMap2 = Mat::zeros(n_col, n_row, CV_32FC3);
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
        this -> global_x = 0;
        this -> global_y = 0;
        intersection_roads.clear();
        //occupancyMap.release();
        occupancyMap2.release();

    }
};

#endif // LAYOUTCOMPONENT_CROSSING_H


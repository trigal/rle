#ifndef LAYOUTCOMPONENT_BUILDING_H
#define LAYOUTCOMPONENT_BUILDING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "LayoutComponent.h"
#include <iostream>
using namespace std;

class LayoutComponent_Building : public LayoutComponent
{
public:

    /**
     * Implementation of pure virtual method 'calculateWeight'
     */
    void calculateComponentScore()
    {
        cout << "Calculating weight of BUILDING component ID: " << component_id << " that belongs to particle ID: " << particle_id << endl;

    }

    /**
     * Implementation of pure virtual method 'componentPerturbation'
     */
    void componentPerturbation()
    {
        cout << "Perturbating BUILDING component ID: " << component_id << " that belongs to particle ID: " << particle_id << endl;
    }

    /**
     * Implementation of pure virtual method 'componentPerturbation'
     */
    void componentPoseEstimation()
    {
        cout << "Propagating and estimating BUILDING component pose. ID: " << component_id << " that belongs to particle ID: " << particle_id << endl;
    }

    /**
     * @brief STUB
     * @return
     */
    double getAlphas()
    {
        return 0;
    }

    /**
     * @brief LayoutComponent_Building constructor
     * @param p_id
     * @param c_id
     * @param c_state
     * @param c_cov
     */
    LayoutComponent_Building(const unsigned int p_id, const unsigned int c_id, const VectorXd& c_state, const MatrixXd& c_cov)
    {
        particle_id = p_id;
        component_id = c_id;
        component_weight = 0;
        component_state = c_state;
        component_cov = c_cov;
        init();
    }

    LayoutComponent_Building()
    {
        particle_id = 0;
        component_id = 0;
        component_weight = 0;
        component_state = VectorXd::Zero(12);
        component_cov = MatrixXd::Zero(12, 12);
        init();
    }

    ~LayoutComponent_Building()
    {
        particle_id = 0;
        component_id = 0;
        component_weight = 0;
        component_state.resize(0);
        component_cov.resize(0, 0);
        init();
    }

    void setBuildings(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
        buildings__cloud_ = cloud;
    }

private:
    ros::NodeHandle node_handle_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr buildings__cloud_;

    struct
    {
        double x;
        double y;
    } prev_position, actual_position;

    void init()
    {
        buildings__cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

};

#endif // LAYOUTCOMPONENT_BUILDING_H

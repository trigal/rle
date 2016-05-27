#ifndef LAYOUTCOMPONENT_BUILDING_H
#define LAYOUTCOMPONENT_BUILDING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <building_detection/Facade.h>
#include <building_detection/FacadesList.h>
#include "../Facade.hpp"
#include "../Edge.hpp"
#include "LayoutComponent.h"
#include <iostream>
#include "ira_open_street_map/latlon_2_xy.h"
#include "ira_open_street_map/snap_particle_xy.h"
#include "ira_open_street_map/getNearBuildings.h"
#include "ira_open_street_map/get_node_coordinates.h"

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
        tf::Stamped<tf::Pose> current_global_pose = Utils::toGlobalFrame(this->getComponentState());
        Utils::Coordinates latlon = Utils::ecef2lla(current_global_pose.getOrigin().x() , current_global_pose.getOrigin().y(), current_global_pose.getOrigin().z());
        get_near_buildings_server_.request.latitude = latlon.latitude;
        get_near_buildings_server_.request.longitude = latlon.longitude;
        get_near_buildings_server_.request.radius = osm_map_radius_;

        edges_->clear();
        //Get OSM map
        if (get_near_buildings_client_.call(get_near_buildings_server_))
        {
            for (size_t i = 0; i < get_near_buildings_server_.response.points.size() - 1; i = i + 2)
            {
                edges_->push_back(edge(get_near_buildings_server_.response.points[i], get_near_buildings_server_.response.points[i + 1], *(this->getParticlePtr())));
            }
        }

        //Calculate score
        double tot_score = 0.0;
        double norm_term = 0.0;

        if (facades_.size() > 0)
        {
            for (size_t i = 0; i < facades_.size(); i++)
            {
                shared_ptr<road_layout_estimation::Facade> f = facades_.at(i);

                f->findCandidates(edges_, scale_factor);
                f->calculateScore(mu_dist, mu_angle, sigma_dist, sigma_angle, weight_dist, weight_angle);

                tot_score += f->score * f->pcl->size();
                norm_term += f->pcl->size();
            }
            component_weight = tot_score / norm_term;
        }
        else
        {
            ROS_INFO_STREAM("NO DETECTION: propagating previous score");
        }

        ROS_INFO_STREAM("Actual score is " << component_weight);
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

    vector<shared_ptr<road_layout_estimation::Facade>>* getFacades()
    {
        return &facades_;
    }

private:
    ros::NodeHandle node_handle_;
    vector<shared_ptr<road_layout_estimation::Facade>> facades_;
    ira_open_street_map::getNearBuildings get_near_buildings_server_;
    ros::ServiceClient get_near_buildings_client_;
    double osm_map_radius_;
    boost::shared_ptr<std::vector<edge>> edges_;
    double scale_factor, mu_dist, mu_angle, sigma_dist, sigma_angle, weight_angle, weight_dist;

    struct
    {
        double x;
        double y;
    } prev_position, actual_position;

    void init()
    {
        facades_;
        edges_.reset(new std::vector<edge>);
        get_near_buildings_client_ = node_handle_.serviceClient<ira_open_street_map::getNearBuildings>("/ira_open_street_map/getNearBuildings");
        get_near_buildings_client_.waitForExistence();
        node_handle_.param("score/scale_factor", scale_factor, 10.0);
        node_handle_.param("score/mu_dist", mu_dist, 0.0);
        node_handle_.param("score/mu_angle", mu_angle, 0.0);
        node_handle_.param("score/sigma_dist", sigma_dist, 2.0);
        node_handle_.param("score/sigma_angle", sigma_angle, 10.0);
        node_handle_.param("score/weight_dist", weight_dist, 0.6);
        node_handle_.param("score/weight_angle", weight_angle, 0.4);
    }

};

#endif // LAYOUTCOMPONENT_BUILDING_H

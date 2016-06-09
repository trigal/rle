#ifndef LAYOUTCOMPONENT_BUILDING_H
#define LAYOUTCOMPONENT_BUILDING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
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

    LayoutComponent* clone()
    {
        LayoutComponent* cloned = new LayoutComponent_RoadLane(*this);
        return cloned;
    }

    tf::Stamped<tf::Pose> stateToGlobalFrame()
    {
        // Get particle state
        Vector3d p_state = this->getParticlePtr()->getParticleState()._pose;
        geometry_msgs::PoseStamped pose_local_map_frame;
        pose_local_map_frame.header.frame_id = "local_map";
        pose_local_map_frame.header.stamp = ros::Time::now();
        pose_local_map_frame.pose.position.x = p_state(0);
        pose_local_map_frame.pose.position.y =  p_state(1);
        pose_local_map_frame.pose.position.z =  p_state(2);

        //Porcata assurda. Funziona solo nel 2D, come tutto il resto
        auto rotation = tf::createQuaternionFromYaw(this->getParticlePtr()->getParticleState().getRotation().angle());
        pose_local_map_frame.pose.orientation.x = rotation.x();
        pose_local_map_frame.pose.orientation.y = rotation.y();
        pose_local_map_frame.pose.orientation.z = rotation.z();
        pose_local_map_frame.pose.orientation.w = rotation.w();

        tf::Stamped<tf::Pose> tf_pose_map_frame, tf_pose_local_map_frame;
        tf::poseStampedMsgToTF(pose_local_map_frame, tf_pose_local_map_frame);

        tf_pose_map_frame.setOrigin(tf::Vector3(0, 0, 0));
        tf_pose_map_frame.setRotation(tf::createIdentityQuaternion());

        // Transform pose from "local_map" to "map"
        try
        {
            tf_listener_.transformPose("map", ros::Time(0), tf_pose_local_map_frame, "local_map", tf_pose_map_frame);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ROS_INFO_STREAM("if(!LayoutManager::first_run){ if(config.particles_number > num_particles)");
            ros::shutdown(); // TODO: handle this, now shutdown requested. augusto debug
            return tf_pose_map_frame;
        }

        return tf_pose_map_frame;
    }


    /**
     * Implementation of pure virtual method 'calculateWeight'
     */
    void calculateComponentScore()
    {
        ROS_DEBUG_STREAM("Calculating weight of [BUILDING] component ID: " << component_id << " that belongs to particle ID: " << particle_id);
        tf::Stamped<tf::Pose> current_global_pose = stateToGlobalFrame();
        Utils::Coordinates latlon = Utils::xy2latlon_helper(current_global_pose.getOrigin().x() , current_global_pose.getOrigin().y(), 32, false);
        get_near_buildings_server_.request.latitude = latlon.latitude;
        get_near_buildings_server_.request.longitude = latlon.longitude;
        get_near_buildings_server_.request.radius = osm_map_radius_;

        edges_->clear();
        //Get OSM map
        if (get_near_buildings_client_.call(get_near_buildings_server_))
        {
            for (size_t i = 0; i < get_near_buildings_server_.response.points.size() - 1; i = i + 2)
            {
                edges_->push_back(edge(get_near_buildings_server_.response.points[i], get_near_buildings_server_.response.points[i + 1], this->stateToGlobalFrame()));
            }
        }

        //Calculate score
        double tot_score = 0.0;
        double norm_term = 0.0;
        facades_cloud_->clear();
        if (facades_.size() > 0)
        {
            for (size_t i = 0; i < facades_.size(); i++)
            {
                shared_ptr<road_layout_estimation::Facade> f = facades_.at(i);

                f->findCandidates(edges_, scale_factor, current_global_pose.getOrigin().x(), current_global_pose.getOrigin().y() );
                f->calculateScore(mu_dist, mu_angle, sigma_dist, sigma_angle, weight_dist, weight_angle);

                tot_score += f->score * f->pcl->size();
                norm_term += f->pcl->size();

                *facades_cloud_ += *(f->pcl);
            }
            component_weight = tot_score / norm_term * getAlphas();

            Eigen::Affine3d particle_transform = Eigen::Affine3d::Identity();
            Vector3d p_state = this->getParticlePtr()->getParticleState()._pose;
            particle_transform.translation() << p_state(0), p_state(1), 0.;
            particle_transform.rotate(this->getParticlePtr()->getParticleState().getRotation());
            pcl::transformPointCloud(*facades_cloud_, *facades_cloud_, particle_transform);

            sensor_msgs::PointCloud2 tmp_facades_cloud;
            pcl::toROSMsg(*facades_cloud_, tmp_facades_cloud);
            tmp_facades_cloud.height = facades_cloud_->height;
            tmp_facades_cloud.width = facades_cloud_->width;
            tmp_facades_cloud.header.frame_id = "local_map";
            tmp_facades_cloud.header.stamp = ros::Time::now();
            cloud_pub_.publish(tmp_facades_cloud);
        }
        else
        {
            ROS_DEBUG_STREAM("[BUILDING] NO DETECTION: propagating previous score");
        }

        ROS_DEBUG_STREAM("[BUILDING] Actual score is " << component_weight);
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
    tf::TransformListener tf_listener_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr facades_cloud_;
    ros::Publisher cloud_pub_;
    struct
    {
        double x;
        double y;
    } prev_position, actual_position;

    void init()
    {
        edges_.reset(new std::vector<edge>);
        facades_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("facades_cloud", 1);
        get_near_buildings_client_ = node_handle_.serviceClient<ira_open_street_map::getNearBuildings>("/ira_open_street_map/getNearBuildings");
        get_near_buildings_client_.waitForExistence();
        node_handle_.param("score/scale_factor", scale_factor, 50.0);
        node_handle_.param("score/mu_dist", mu_dist, 0.0);
        node_handle_.param("score/mu_angle", mu_angle, 0.0);
        node_handle_.param("score/sigma_dist", sigma_dist, 2.0);
        node_handle_.param("score/sigma_angle", sigma_angle, 10.0);
        node_handle_.param("score/weight_dist", weight_dist, 0.6);
        node_handle_.param("score/weight_angle", weight_angle, 0.4);
        node_handle_.param("get_map/radius", osm_map_radius_, 35.0);
    }

};

#endif // LAYOUTCOMPONENT_BUILDING_H

#ifndef LAYOUTCOMPONENT_BUILDING_H
#define LAYOUTCOMPONENT_BUILDING_H

#include <boost/math/distributions/normal.hpp>

#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
//#include <point_cloud_registration.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/distances.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
// #include <sensor_msgs/PointCloud2.h>
#include <building_detection/Facade.h>
#include <building_detection/FacadesList.h>
#include "visualization_msgs/Marker.h"
#include <visualization_msgs/MarkerArray.h>
#include "../Facade.hpp"
#include "../Edge.hpp"
#include "LayoutComponent.h"
#include <iostream>
#include "ira_open_street_map/latlon_2_xy.h"
#include "ira_open_street_map/snap_particle_xy.h"
#include "ira_open_street_map/getNearBuildings.h"
#include "ira_open_street_map/get_node_coordinates.h"

#include "Particle.h"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>



using namespace std;
using boost::math::normal;

class LayoutComponent_Building : public LayoutComponent
{
public:
   double meanError(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
    assert(cloud1->size() == cloud2->size());
    double mse = 0;
    for (int i = 0; i < cloud1->size(); i++) {
        mse += euclideanDistance(cloud1->at(i), cloud2->at(i));
    }
    mse /= cloud1->size();
    return mse;
}

   double sumSquaredError(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud1,
                              pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud2)
{
    double sum = 0;
    double median_distance;
    pcl::KdTreeFLANN<pcl::PointXYZRGBL> kdtree;
    kdtree.setInputCloud(cloud2);
    for (std::size_t i = 0; i < cloud1->size(); i++) {
        std::vector<int> neighbours;
        std::vector<float> distances;
        neighbours.reserve(1);
        distances.reserve(1);
        kdtree.nearestKSearch(*cloud1, i, 1, neighbours, distances);
        sum += distances[0];
    }
    return sum;
}
   double robustSumSquaredError(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud1,
                                    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud2)
{
    double sum = 0;
    double median_distance;
    pcl::KdTreeFLANN<pcl::PointXYZRGBL> kdtree;
    kdtree.setInputCloud(cloud2);
    std::vector<double> all_distances;
    for (std::size_t i = 0; i < cloud1->size(); i++) {
        std::vector<int> neighbours;
        std::vector<float> distances;
        neighbours.reserve(1);
        distances.reserve(1);
        kdtree.nearestKSearch(*cloud1, i, 1, neighbours, distances);
        all_distances.push_back(distances[0]);
    }

    std::sort(all_distances.begin(), all_distances.end());
    if (all_distances.size() % 2 != 0) {
        median_distance = all_distances[(all_distances.size() + 1) / 2];
    } else {
        median_distance = (all_distances[all_distances.size() / 2] + all_distances[(all_distances.size() /
                                                                                    2) + 1]) / 2.0;
    }
    int num_filtered = 0;
    for (auto it = all_distances.begin(); it != all_distances.end(); it++) {
        if (*it <= median_distance * 3 && *it >= median_distance / 3) {
            sum += (*it);
            num_filtered++;
        }
    }
    if (num_filtered < 10) {
        return std::numeric_limits<double>::max();
    }
    return sum/num_filtered;
}
   double robustMedianClosestDistance(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud1,
                                          pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud2)
{
    double median_distance = 0;
    pcl::KdTreeFLANN<pcl::PointXYZRGBL> kdtree;
    kdtree.setInputCloud(cloud2);
    std::vector<float> distances;
    std::vector<float> filtered_distances;
    for (std::size_t i = 0; i < cloud1->size(); i++) {
        std::vector<int> neighbours;
        std::vector<float> dist;
        neighbours.reserve(1);
        distances.reserve(1);
        kdtree.nearestKSearch(*cloud1, i, 1, neighbours, dist);
        distances.push_back(dist[0]);
    }
    std::sort(distances.begin(), distances.end());
    if (distances.size() % 2 != 0) {
        median_distance = distances[(distances.size() + 1) / 2];
    } else {
        median_distance = (distances[distances.size() / 2] + distances[(distances.size() / 2) + 1]) / 2.0;
    }
    for (auto it = distances.begin(); it != distances.end(); it++) {
        if (*it <= median_distance * 3 && *it >= median_distance / 3.0) {
            filtered_distances.push_back(*it);
        }
    }
    if (filtered_distances.size() % 2 != 0) {
        median_distance = filtered_distances[(filtered_distances.size() + 1) / 2];
    } else {
        median_distance = (filtered_distances[filtered_distances.size() / 2] +
                           filtered_distances[(filtered_distances.size() / 2) + 1]) / 2.0;
    }
    return median_distance / (filtered_distances.size()*filtered_distances.size());
}

 double medianClosestDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
    double median_distance = 0;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud2);
    std::vector<float> distances;
    for (std::size_t i = 0; i < cloud1->size(); i++) {
        std::vector<int> neighbours;
        std::vector<float> dist;
        neighbours.reserve(1);
        distances.reserve(1);
        kdtree.nearestKSearch(*cloud1, i, 1, neighbours, dist);
        distances.push_back(dist[0]);
    }
    std::sort(distances.begin(), distances.end());
    if (distances.size() % 2 != 0) {
        median_distance = distances[(distances.size() + 1) / 2];
    } else {
        median_distance = (distances[distances.size() / 2] + distances[(distances.size() / 2) + 1]) / 2.0;
    }
    return median_distance;
}


   double robustAveragedSumSquaredError(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud1,
                                            pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud2)
{
    double sum = 0;
    double median_distance;
    pcl::KdTreeFLANN<pcl::PointXYZRGBL> kdtree;
    kdtree.setInputCloud(cloud2);
    std::vector<double> all_distances;
    for (std::size_t i = 0; i < cloud1->size(); i++) {
        std::vector<int> neighbours;
        std::vector<float> distances;
        neighbours.reserve(1);
        distances.reserve(1);
        kdtree.nearestKSearch(*cloud1, i, 1, neighbours, distances);
        all_distances.push_back(distances[0]);
    }

    std::sort(all_distances.begin(), all_distances.end());
    if (all_distances.size() % 2 != 0) {
        median_distance = all_distances[(all_distances.size() + 1) / 2];
    } else {
        median_distance = (all_distances[all_distances.size() / 2] + all_distances[(all_distances.size() /
                                                                                    2) + 1]) / 2.0;
    }
    int num_filtered = 0;
    for (auto it = all_distances.begin(); it != all_distances.end(); it++) {
        if (*it <= median_distance * 3 && *it >= median_distance / 3) {
            sum += (*it);
            num_filtered++;
        }
    }
    if (num_filtered < 10) {
        return std::numeric_limits<double>::max();
    }
    return sum / num_filtered;
}

  std::string exec_my_command(const char* cmd) {
      std::array<char, 128> buffer;
      std::string result;
      std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
      if (!pipe) throw std::runtime_error("popen() failed!");
      while (!feof(pipe.get())) {
          if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
              result += buffer.data();
      }
      return result;
  }

  void setFacades_cloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud){
    facades_cloud_ = cloud;
  }


    //  PCL computation time
    pcl::console::TicToc tt;

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
        //auto rotation = tf::createQuaternionFromYaw(this->getParticlePtr()->getParticleState().getRotation().angle());
        auto rotation = tf::createQuaternionFromRPY(this->getParticlePtr()->getParticleState().getRoll(),
                                                    this->getParticlePtr()->getParticleState().getPitch(),
                                                    this->getParticlePtr()->getParticleState().getYaw());
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
            tf_listener_->transformPose("map", ros::Time(0), tf_pose_local_map_frame, "local_map", tf_pose_map_frame);
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

    visualization_msgs::Marker publish_box(double x, double y, int id)
    {
        visualization_msgs::Marker m1;
        uint32_t shape = visualization_msgs::Marker::SPHERE;
        m1.header.frame_id = "local_map";
        m1.id = id;
        m1.type = shape;
        m1.action = visualization_msgs::Marker::ADD;
        m1.pose.position.x = x;
        m1.pose.position.y = y;
        m1.pose.position.z = 0;
        m1.pose.orientation.x = 0.0;
        m1.pose.orientation.y = 0.0;
        m1.pose.orientation.z = 0.0;
        m1.pose.orientation.w = 1.0;
        m1.scale.x = 1.;
        m1.scale.y = 1.;
        m1.scale.z = 1.;
        m1.color.r = 0.0f;
        m1.color.g = 0.0f;
        m1.color.b = 1.0f;
        m1.color.a = 1.0f;
        m1.header.stamp = ros::Time::now();
        return m1;
    }

    void calculateComponentScore()
    {
        tt.tic();
        ROS_DEBUG_STREAM("Calculating weight of [BUILDING] component ID: " << component_id << " that belongs to particle ID: " << particle_id);
        tf::Stamped<tf::Pose> current_global_pose = stateToGlobalFrame();
        Utils::Coordinates latlon = Utils::xy2latlon_helper(current_global_pose.getOrigin().x() , current_global_pose.getOrigin().y(), 32, false);
        get_near_buildings_server_.request.latitude = latlon.latitude;
        get_near_buildings_server_.request.longitude = latlon.longitude;
        //get_near_buildings_server_.request.theta = this->getParticlePtr()->getParticleState().getYaw();
        get_near_buildings_server_.request.theta = 0.;
        get_near_buildings_server_.request.radius = osm_map_radius_;

        ROS_DEBUG_STREAM("[BUILDING] pose: " << this->stateToGlobalFrame().getOrigin().getX() << " , " << this->stateToGlobalFrame().getOrigin().getY());

        edges_->clear();
        visualization_msgs::Marker buildings;
        buildings.header.frame_id = "local_map";
        buildings.header.stamp = ros::Time();
        buildings.ns = "buildings";
        buildings.id = 0;
        buildings.type = visualization_msgs::Marker::TRIANGLE_LIST;
        buildings.action = visualization_msgs::Marker::ADD;
        buildings.scale.x = 1.0;
        buildings.scale.y = 1.0;
        buildings.scale.z = 1.0;
        buildings.color.r = 1.0f;
        buildings.color.g = 0.9f;
        buildings.color.b = 0.7f;
        buildings.color.a = 0.75f;

        float height = 3.0f;
        geometry_msgs::Point h_A, h_B, A, B;
        std::stringstream edge_cloud_name;

        //Get OSM map
        std::string id = lexical_cast<std::string>((random_generator())());
        if (get_near_buildings_client_.call(get_near_buildings_server_))
        {
            /*geometry_msgs::Point p1, p2, p3, p4;
            p1.x = get_near_buildings_server_.response.p1x;
            p1.y = get_near_buildings_server_.response.p1y;
            p2.x = get_near_buildings_server_.response.p2x;
            p2.y = get_near_buildings_server_.response.p2y;
            p3.x = get_near_buildings_server_.response.p3x;
            p3.y = get_near_buildings_server_.response.p3y;
            p4.x = get_near_buildings_server_.response.p4x;
            p4.y = get_near_buildings_server_.response.p4y;
            edge bound_edge(p1, p2, this->stateToGlobalFrame());
            marker_array.markers.push_back(publish_box(bound_edge.A[0], bound_edge.A[1], 0));
            marker_array.markers.push_back(publish_box(bound_edge.B[0], bound_edge.B[1], 1));
            edge bound_edge2(p3, p4, this->stateToGlobalFrame());
            marker_array.markers.push_back(publish_box(bound_edge2.A[0], bound_edge2.A[1], 2));
            marker_array.markers.push_back(publish_box(bound_edge2.B[0], bound_edge2.B[1], 3));*/
            edges_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());

            for (size_t i = 0; i < get_near_buildings_server_.response.points.size() - 1; i = i + 2)
            {
                //QUESTE DUE SERVONO
                edge temp_edge(get_near_buildings_server_.response.points[i], get_near_buildings_server_.response.points[i + 1], this->stateToGlobalFrame());
                edges_->push_back(temp_edge);
                *edges_cloud_+=(*temp_edge.cloud);
            }

            // if (edges_cloud_->size() > 0)
            // {
            //     edge_cloud_name << "/tmp/edge_" << id << ".pcd";
            //     pcl::io::savePCDFile(edge_cloud_name.str(), *edges_cloud_);
            // }

            sensor_msgs::PointCloud2 tmp_edge_cloud;
            pcl::toROSMsg(*edges_cloud_, tmp_edge_cloud);
            tmp_edge_cloud.height = edges_cloud_->height;
            tmp_edge_cloud.width = edges_cloud_->width;
            tmp_edge_cloud.header.frame_id = "local_map";
            tmp_edge_cloud.header.stamp = ros::Time::now();
            cloud_pub_.publish(tmp_edge_cloud);
            edge_pub_.publish(tmp_edge_cloud);
        }

        //Calculate score
        double tot_score = 0.0;
        double norm_term = 0.0;
        // facades_cloud_->clear();
        //if ((facades_cloud_->size() > 0 && edges_cloud_->size()>0) && (0)) // TOGLIMI !!! AUGUSTO ZIOCANE QUESTO ZERO
        if (facades_cloud_->size() > 0 && edges_cloud_->size()>0)
        {



          //prendi la rotazione della particella associata a questa istanza di componente.
          double angle = tf::getYaw(this->stateToGlobalFrame().getRotation());
          Eigen::Affine3f transform;   // rotation for having x-forward
          transform = Eigen::Affine3f::Identity(); //reset transform
          transform.rotate (Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitZ()));
//          pcl::transformPointCloud (*facades_cloud_, *facades_cloud_, transform);

          transform = Eigen::Affine3f::Identity(); //reset transform before TRANSLATE
          Vector3d v = this->particlePtr->getParticleState().getPosition();
          transform.translation() << -v[0] , -v[1] , 0.f;
          pcl::transformPointCloud (*facades_cloud_, *facades_cloud_, transform);


//          transform = Eigen::Affine3f::Identity(); //reset transform before ROTATE
//          transform.rotate (Eigen::AngleAxisf (this->particlePtr->getParticleState().getYaw() , Eigen::Vector3f::UnitZ()));
//          pcl::transformPointCloud (*facades_cloud_, *facades_cloud_, transform);

          // std::stringstream facade_cloud_name;
          // facade_cloud_name << "/tmp/facade_" << id << ".pcd";
          // pcl::io::savePCDFile(facade_cloud_name.str(), *facades_cloud_);

          // if (component_weight < 0.0001)
          //     component_weight = 0.000001f;

          // if (0) /////augusto
          // {
            pcl::console::TicToc forannidati;
            forannidati.tic();

            // char command[500];
            // // sprintf (command, "convert -in %s -out %s -n -e -h", myFile, convertedFile);
            // sprintf(command, "/home/simone/Documenti/point-cloud-registration/build/icp_registration %s %s -t 0.5 -s 0.5 -r 1 -u",facade_cloud_name.str().c_str(), edge_cloud_name.str().c_str());
            // ROS_ERROR_STREAM(command);
            // std::string result = exec_my_command(command);
            // double score = stod(result);
            // ROS_ERROR_STREAM(score);
            // normal gaussian(0,2);
            // component_weight = pdf(gaussian, score);
            // ROS_ERROR_STREAM(component_weight<<std::endl<<std::endl);
            // component_weight = component_weight * getAlphas();
            // component_weight = getAlphas();

            pcl::VoxelGrid<pcl::PointXYZRGBL> sor_rgbl;
            sor_rgbl.setInputCloud (facades_cloud_);
            sor_rgbl.setLeafSize (0.5f, 0.5f, 0.5f);
            sor_rgbl.filter (*facades_cloud_);
            pcl::VoxelGrid<pcl::PointXYZ> sor_xyz;
            sor_xyz.setInputCloud (edges_cloud_);
            sor_xyz.setLeafSize (0.5f, 0.5f, 0.5f);
            sor_xyz.filter (*edges_cloud_);
            // pcl::KdTreeFLANN<pcl::PointXYZRGBL> kdtree;
            // std::vector< int > index(1);
            // std::vector< float > distance(1);
            // kdtree.setInputCloud (edges_cloud_);
            // double score = 0;
            // int n_points = 0;
            // for(auto point: facades_cloud_->points){
            //   kdtree.nearestKSearch(point, 1,index, distance);
            //   if(distance[0]<2){
            //     score += distance[0];
            //     n_points++;
            //   }
            // }
            // ROS_ERROR_STREAM("Registration score: "<<score<<std::endl<<std::endl);
            // score /= n_points;
            // score = sumSquaredError(facades_cloud_, edges_cloud_)/facades_cloud_->size();
            pcl::PointCloud<pcl::PointXYZ>::Ptr facades_xyz = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            copyPointCloud(*facades_cloud_, *facades_xyz);
//             pcl::PassThrough<pcl::PointXYZ> pass;
//             pass.setInputCloud (facades_xyz);
//             pass.setFilterFieldName ("x");
//             pass.setFilterLimits (-30, 6);
// // //pass.setFilterLimitsNegative (true);
//             pass.filter (*facades_xyz);
//             pass.setFilterFieldName ("y");
//             pass.setFilterLimits (0.0, 5.0);
//             pass.filter (*facades_xyz);
              double score = 0;
if(facades_xyz->size()>30){
            double median = medianClosestDistance(facades_xyz, edges_cloud_);
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
            gicp.setMaxCorrespondenceDistance(3*median);
            // gicp.setMaxCorrespondenceDistance(10);
            gicp.setMaximumIterations(10);
            gicp.setInputSource(facades_xyz);
            gicp.setInputTarget(edges_cloud_);
            pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_source = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            gicp.align(*aligned_source);
            score = meanError(facades_xyz, aligned_source);
            if(gicp.hasConverged()==false || std::isnan(score) ){
              score = 100000000;
            }
          }
          else{
            score = 0.0000000000000001;
          }
            ROS_ERROR_STREAM("Registration score: "<<score<<std::endl<<std::endl);
            normal gaussian(0,100);
            component_weight = pdf(gaussian, score)/ pdf(gaussian, 0) * getAlphas();
            if (component_weight < 0.0000000000000001){
                component_weight = 0.0000000000000001;
              }
            ROS_ERROR_STREAM("Weight score: "<<component_weight<<std::endl<<std::endl);

//             for (size_t i = 0; i < facades_.size(); i++)
//             {
//                 road_layout_estimation::Facade f = facades_.at(i);
//
//                 f.findCandidates(edges_, scale_factor, current_global_pose.getOrigin().x(), current_global_pose.getOrigin().y() );
//                 f.calculateScore(mu_dist, mu_angle, sigma_dist, sigma_angle, weight_dist, weight_angle);
//
//                 tot_score += f.score * f.pcl->size();
//                 norm_term += f.pcl->size();
//
// //                pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_pcl;
// //                temp_pcl.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
// //                pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZRGB>(*(f.pcl), *temp_pcl);
// //                for (int j = 0; j < temp_pcl->size(); j++)
// //                {
// //                    if (i % 3 == 0)
// //                    {
// //                        temp_pcl->at(j).r = 255;
// //                    }
// //                    if (i % 3 == 1)
// //                    {
// //                        temp_pcl->at(j).g = 255;
// //                    }
// //                    if (i % 3 == 2)
// //                    {
// //                        temp_pcl->at(j).b = 255;
// //                    }
// //                }
//
// //                *facades_cloud_ += *temp_pcl;
//             }


            double ttt2=forannidati.toc();
            if (ttt2>1)
                ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " forannidati " << ttt2 << " milliseconds");

            /*Eigen::Affine3d particle_transform = Eigen::Affine3d::Identity();
            Vector3d p_state = this->getParticlePtr()->getParticleState()._pose;
            particle_transform.translation() << p_state(0), p_state(1), 0.;
            particle_transform.rotate(this->getParticlePtr()->getParticleState().getRotation());
            pcl::transformPointCloud(*facades_cloud_, *facades_cloud_, particle_transform);*/

//            sensor_msgs::PointCloud2 tmp_facades_cloud;
//            pcl::toROSMsg(*facades_cloud_, tmp_facades_cloud);
//            tmp_facades_cloud.height = facades_cloud_->height;
//            tmp_facades_cloud.width = facades_cloud_->width;
//            tmp_facades_cloud.header.frame_id = "local_map";
//            tmp_facades_cloud.header.stamp = ros::Time::now();
//            cloud_pub_.publish(tmp_facades_cloud);



            // }
        }
        else
        {
            ROS_DEBUG_STREAM("[BUILDING] NO DETECTION: propagating previous score");
            component_weight=component_weight*0.5;

            if (component_weight < 0.00000001)
                component_weight = 0.00000001f;


        }

        double ttt=tt.toc();
        if (ttt>1)
            ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ttt << " milliseconds");
        ROS_ERROR_STREAM("[BUILDING] Actual score is " << component_weight);
    }

    /**
     * Implementation of pure virtual method 'componentPerturbation'
     */
    void componentPerturbation()
    {
        ROS_DEBUG_STREAM("Entering > " << __PRETTY_FUNCTION__);
        cout << "Perturbating BUILDING component ID: " << component_id << " that belongs to particle ID: " << particle_id << endl;
    }

    /**
     * Implementation of pure virtual method 'componentPerturbation'
     */
    void componentPoseEstimation(int index)
    {
        ROS_DEBUG_STREAM("Entering > " << __PRETTY_FUNCTION__);
        cout << "Propagating and estimating BUILDING component pose. ID: " << component_id << " that belongs to particle ID: " << particle_id << endl;
    }

    /**
     * @brief STUB
     * @return
     */
    double getAlphas()
    {
        return 10;
    }

    /**
     * @brief LayoutComponent_Building constructor
     * @param p_id
     * @param c_id
     * @param c_state
     * @param c_cov
     */

    LayoutComponent* clone()
    {
        LayoutComponent* cloned = new LayoutComponent_Building(*this);
        return cloned;
    }

    LayoutComponent_Building(ros::NodeHandle nh, tf::TransformListener *listener, const unsigned int p_id, const unsigned int c_id, const VectorXd& c_state, const MatrixXd& c_cov): node_handle_(nh)
    {
        tf_listener_=listener;
        particle_id = p_id;
        component_id = c_id;
        component_weight = 0;
        component_state = c_state;
        component_cov = c_cov;
        init();
    }

    LayoutComponent_Building(ros::NodeHandle nh,tf::TransformListener *listener): node_handle_(nh)
    {
        tf_listener_=listener;
        particle_id = 0;
        component_id = 0;
        component_weight = 0;
        component_state = VectorXd::Zero(12);
        component_cov = MatrixXd::Zero(12, 12);
        init();
    }

    LayoutComponent_Building(LayoutComponent_Building& component): node_handle_(component.node_handle_), tf_listener_(component.tf_listener_)
    {
        particle_id = component.particle_id;
        component_id = component.component_id;
        component_weight = component.component_weight;
        component_state = component.component_state;
        component_cov = component.component_cov;
        init();
        facades_cloud_ = component.facades_cloud_;
        *edges_ = *(component.edges_);
    }

    ~LayoutComponent_Building()
    {
        node_handle_.shutdown();

    }

    vector<road_layout_estimation::Facade>* getFacades()
    {
        return &facades_;
    }

private:
    ros::NodeHandle node_handle_;
    tf::TransformListener *tf_listener_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr edges_cloud_;
    vector<road_layout_estimation::Facade> facades_;
    ira_open_street_map::getNearBuildings get_near_buildings_server_;
    ros::ServiceClient get_near_buildings_client_;
    double osm_map_radius_;
    boost::shared_ptr<std::vector<edge>> edges_;
    double scale_factor, mu_dist, mu_angle, sigma_dist, sigma_angle, weight_angle, weight_dist;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr facades_cloud_;
    ros::Publisher cloud_pub_;
    ros::Publisher edge_pub_;
    struct
    {
        double x;
        double y;
    } prev_position, actual_position;

    void init()
    {
        edges_.reset(new std::vector<edge>);
        facades_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBL>);
        cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("facades_cloud", 1);
        edge_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/edges", 1);
        get_near_buildings_client_ = node_handle_.serviceClient<ira_open_street_map::getNearBuildings>("/ira_open_street_map/getNearBuildings");
        get_near_buildings_client_.waitForExistence();
        node_handle_.param("score/scale_factor", scale_factor, 50.0);
        node_handle_.param("score/mu_dist", mu_dist, 0.0);
        node_handle_.param("score/mu_angle", mu_angle, 0.0);
        node_handle_.param("score/sigma_dist", sigma_dist, 1.);
        node_handle_.param("score/sigma_angle", sigma_angle, 0.03);
        node_handle_.param("score/weight_dist", weight_dist, 0.6);
        node_handle_.param("score/weight_angle", weight_angle, 0.4);
        node_handle_.param("get_map/radius", osm_map_radius_, 35.0);
    }

};

#endif // LAYOUTCOMPONENT_BUILDING_H

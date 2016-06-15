#ifndef FACADE_H
#define FACADE_H

#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <building_detection/Facade.h>
#include "Edge.hpp"
#include <boost/math/distributions/normal.hpp>

namespace road_layout_estimation
{
struct Facade
{
    //  PCL computation time
    pcl::console::TicToc tt;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl;
    Eigen::Vector3d abc;
    double d;
    std::vector<edge> candidates;
    int num_candidates = 0;
    int best_match;
    std::vector<double> scores, scores_dist, scores_angle;
    double score;
    pcl::PointXYZ centroid;

    Facade(pcl::PointCloud<pcl::PointXYZ>::Ptr _pcl, pcl::ModelCoefficients::Ptr _coeff, pcl::PointXYZ _centroid)
    {
        pcl = _pcl;
        abc = Eigen::Vector3d(_coeff->values[0], _coeff->values[1], _coeff->values[2]);
        d = _coeff->values[3];
        centroid = _centroid;
    }

    Facade(building_detection::Facade& facade_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(facade_msg.cloud, *cloud);
        pcl = cloud;
        abc = Eigen::Vector3d(facade_msg.a, facade_msg.b, facade_msg.c);
        centroid = pcl::PointXYZ(facade_msg.centroid.x, facade_msg.centroid.y, facade_msg.centroid.z);
    }

    bool isClose(edge e, double scale_factor, double current_pose_x, double current_pose_y)
    {
        tt.tic();
        bool is_close = false;
        double min_dist = DBL_MAX;
        for (size_t n = 0; n < pcl->size(); n++)
        {
            double dist = sqrt(((pcl->at(n).x) - e.centroid[0]) * ((pcl->at(n).x) - e.centroid[0]) +
                               ((pcl->at(n).y) - e.centroid[1]) * ((pcl->at(n).y) - e.centroid[1]));
            if (dist < min_dist) min_dist = dist;
        }

        if (min_dist <= e.length * scale_factor)
        {
            is_close = true;
        }
        double ttt=tt.toc();
        if (ttt>1)
            ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ttt << " milliseconds");
        return is_close;

    }

    void findCandidates(boost::shared_ptr<std::vector<edge>> _edges, double _scale_factor, double current_pose_x, double current_pose_y)
    {        
        pcl::console::TicToc tt;
        tt.tic();
        candidates.clear();
        for (size_t i = 0; i < _edges->size(); i++)
        {
            edge e = _edges->at(i);

            if (this->isClose(e, _scale_factor, current_pose_x, current_pose_y))
            {
                candidates.push_back(edge(e));
                num_candidates++;
            }
        }
        scores.reserve(candidates.size());
        scores_dist.reserve(candidates.size());
        scores_angle.reserve(candidates.size());

        double ttt=tt.toc();
        if (ttt>1)
            ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ttt << " milliseconds");

    }

    void calculateScore(double _mu_dist, double _mu_angle, double _sigma_dist, double _sigma_angle, double _weight_dist, double _weight_angle)
    {
        pcl::console::TicToc tt;
        tt.tic();
        double max_score = 0.0;
        boost::math::normal nd_dist(_mu_dist, _sigma_dist); //10
        boost::math::normal nd_angle(_mu_angle, _sigma_angle); //10

        if (candidates.size()==0)
        {
            score = max_score;
            return;
        }

        ROS_DEBUG_STREAM("[BUILDING] Candidate Size: " << candidates.size());
        best_match = 0;
        for (size_t i = 0; i < candidates.size(); i++)
        {
            scores_dist[i] = scoreDist(candidates[i], _mu_dist, _sigma_dist);
            scores_angle[i] = scoreAngle(candidates[i], _mu_angle, _sigma_angle);

            //scores_angle[i] = abs(scores_angle[i]);

            scores[i] = _weight_dist * (pdf(nd_dist, scores_dist[i]) / pdf(nd_dist, _mu_dist)) +
                        _weight_angle * (pdf(nd_angle, scores_angle[i]) / pdf(nd_angle, _mu_angle));

            if (scores[i] > max_score)
            {
                max_score = scores[i];
                best_match = i;
            }
        }

        //ROS_DEBUG_STREAM("[BUILDING] Angle Diff: " << scores_angle[best_match]);
        //ROS_DEBUG_STREAM("[BUILDING] Avg Distance: " << scores_dist[best_match]);
        //ROS_DEBUG_STREAM("[BUILDING] Best Edge (centroid) : " << candidates.at(best_match).centroid.x<< " "<<candidates.at(best_match).centroid.y << " "<< candidates.at(best_match).centroid.z);
        ROS_DEBUG_STREAM("[BUILDING] Best Edge (centroid) : " << candidates.at(best_match).centroid[0] << " "<< candidates.at(best_match).centroid[1] << " "<< candidates.at(best_match).centroid[2]);

        score = max_score;
        double ttt=tt.toc();
        if (ttt>1)
            ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ttt << " milliseconds");
    }

    double scoreAngle(edge candidate, double mu, double sigma)
    {

        double angle = acos(abc.dot(candidate.abc) / (abc.norm() * candidate.abc.norm()));

        //ROS_DEBUG_STREAM("[BUILDING] Angle Diff: "<< angle);

        //return pdf(nd, angle) / pdf(nd, mu);
        return angle;
    }

    double scoreDist(edge candidate, double mu, double sigma)
    {
        tt.tic();
        double avg_dist = 0;

        for (size_t i = 0; i < pcl->size(); i++)
        {
            double single_dist = fabs(pcl->at(i).x * candidate.abc[0] +
                                      pcl->at(i).y * candidate.abc[1] +
                                      pcl->at(i).z * candidate.abc[2] +
                                      candidate.d) / candidate.abc.norm();
            avg_dist += single_dist;
        }
        avg_dist /= pcl->size();

        //ROS_DEBUG_STREAM("[BUILDING] Avg Distance: "<<avg_dist);

        //return pdf(nd, avg_dist) / pdf(nd, mu);
        double ttt=tt.toc();
        if (ttt>1)
            ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ttt << " milliseconds");
        return avg_dist;

    }


};
}
#endif // FACADE_H

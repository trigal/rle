/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Author:    Dario Limongi                                              *
 *   Email:     dario.limongi@gmail.com                                    *
 *   Date:      25/05/2014                                                 *
 *                                                                         *
 ***************************************************************************/

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "Utils.h"
#include "particle/State6DOF.h"

#include <Eigen/Dense>	//used for motion threshold matrix
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

using namespace Eigen;

class MeasurementModel {

private:
//    VectorXd msr_state; /// measurement (12x1: 6DoF pose + 6 Speed Derivates)
    MatrixXd _msr_cov;   /// measurment covariance (12x12)
    State6DOF _measure_Delta;
    tf::TransformListener *_listener;
    tf::StampedTransform _old_transform;
    tf::StampedTransform fixed_transform; //fixed transform from ODOM(libviso) to VISUAL_ODOMETRY_X_FORWARD
    nav_msgs::Odometry _msg, _old_msg; /// current msg arrived from Visual Odometry

    geometry_msgs::Twist frameSpeed;
    geometry_msgs::Twist oldFrameSpeed;
    ros::Duration delta_time;

    class Tracker{

    };

    class Mapper{

    };

public:

    bool measurementModelFirstRunNotExecuted;
    /**
     * @brief measurementJacobi
     * @param p_state_predicted
     * @return
     */
    MatrixXd measurementJacobian(State6DOF &p_state_predicted);

    /**
     * @param p_state
     * @return measurement vector of the given particle, obtained using measurement model equations
     */
    State6DOF measurePose(State6DOF &p_state);


    // getters & setters --------------------------------------------------------------------------
    void setMeasureState(State6DOF& mrs)
    {
        //        _measure_Delta._pose = mrs._pose;
        //        _measure_Delta._rotation = mrs._rotation;
        //        _measure_Delta._translational_velocity= mrs._translational_velocity;
        //        _measure_Delta._rotational_velocity= mrs._rotational_velocity;
        _measure_Delta.setPose(mrs.getPose());
        _measure_Delta.setRotation(mrs.getRotation());
        _measure_Delta.setTranslationalVelocity(mrs.getTranslationalVelocity());
        _measure_Delta.setRotationalVelocity(mrs.getRotationalVelocity());
    }

    State6DOF getOrthogonalMeasureDeltaState()
    {
        State6DOF orthogonalDelta;
        orthogonalDelta = _measure_Delta;
        orthogonalDelta.setOrthogonalPoseRotation();
        orthogonalDelta.setOrthogonalSpeedRotation();
        _measure_Delta.printState("Original delta state");
        orthogonalDelta.printState("Orthogonal delta state");
        return orthogonalDelta;
    }

    State6DOF getMeasureDeltaState()
    {
        return _measure_Delta;
    }

    State6DOF getMeasureDeltaStateScaledWithTime(double deltaTimerTime=0.0f, double deltaOdomTime=0.0f)
    {
        /// Here the _measure_delta is scaled by the fraction of time between the last libviso2 delta and the elapsed time from the last iteration of RLE

        double scaling_factor=deltaOdomTime/deltaTimerTime;
        //ROS_ASSERT(scaling_factor>0);
        //ROS_ASSERT(std::isnormal(scaling_factor));  //a normal value: i.e., whether it is neither infinity, NaN, zero or subnormal.

        State6DOF scaled;
        Vector3d tmpVector3d;
        AngleAxisd tmpAngleAxisd;

        tmpVector3d =_measure_Delta.getPose();
        tmpAngleAxisd =_measure_Delta.getRotation();

        tmpVector3d(0)/=scaling_factor;
        tmpVector3d(1)/=scaling_factor;
        tmpVector3d(2)/=scaling_factor;
        tmpAngleAxisd.angle()=tmpAngleAxisd.angle()/scaling_factor;

        scaled.setPose(tmpVector3d);
        scaled.setRotation(tmpAngleAxisd);
        scaled.setRotationalVelocity(_measure_Delta.getRotationalVelocity());
        scaled.setTranslationalVelocity(_measure_Delta.getTranslationalVelocity());

        //_measure_Delta.printState("ORIGINAL");
        //scaled.printState("SCALED");

        //ROS_ASSERT(scaled.getRotation().isUnitary());
        //ROS_ASSERT(scaled.getRotationalVelocity().isUnitary());

        return scaled;
    }


    //void setMeasureCov(MatrixXd& msrcov){ _msr_cov = msrcov; }
    //void setMeasureCov(double unc){ _msr_cov = MatrixXd::Identity(12,12) * (unc*unc); }
    void setMeasureCov(double position_uncertainty, double orientation_uncertainty, double speed_linear_uncertainty, double speed_angular_uncertainty)
    {
        ROS_ERROR_STREAM("setMeasureCov" << "\t" << position_uncertainty << "\t" << orientation_uncertainty << "\t" << speed_linear_uncertainty << "\t" << speed_angular_uncertainty);

        _msr_cov = MatrixXd::Zero(12,12);
        _msr_cov(0,0)   = position_uncertainty      * position_uncertainty;
        _msr_cov(1,1)   = position_uncertainty      * position_uncertainty;
        _msr_cov(2,2)   = position_uncertainty      * position_uncertainty;
        _msr_cov(3,3)   = orientation_uncertainty   * orientation_uncertainty;
        _msr_cov(4,4)   = orientation_uncertainty   * orientation_uncertainty;
        _msr_cov(5,5)   = orientation_uncertainty   * orientation_uncertainty;
        _msr_cov(6,6)   = speed_linear_uncertainty  * speed_linear_uncertainty;
        _msr_cov(7,7)   = speed_linear_uncertainty  * speed_linear_uncertainty;
        _msr_cov(8,8)   = speed_linear_uncertainty  * speed_linear_uncertainty;
        _msr_cov(9,9)   = speed_angular_uncertainty * speed_angular_uncertainty;
        _msr_cov(10,10) = speed_angular_uncertainty * speed_angular_uncertainty;
        _msr_cov(11,11) = speed_angular_uncertainty * speed_angular_uncertainty;
    }
    MatrixXd getMeasureCov(){ return _msr_cov; }


    bool getFirstRun() {return measurementModelFirstRunNotExecuted;}
    void setMsg(const nav_msgs::Odometry& odometryMessage);

    nav_msgs::Odometry getMsg() { return _msg; }
    nav_msgs::Odometry getOldMsg() { return _old_msg; }

    // constructor & destructor -------------------------------------------------------------------
    MeasurementModel()
    {
        // Sets all values to zero
        _msr_cov = MatrixXd::Zero(12,12);
        _listener = new tf::TransformListener();
        measurementModelFirstRunNotExecuted = true;
    }

    ~MeasurementModel(){
        // Resize all values to zero
        _msr_cov.resize(0,0);
        delete _listener;
    }
    tf::StampedTransform getFixed_transform() const;
    void setFixed_transform(const tf::StampedTransform &value);
    ros::Duration getDelta_time() const;
    void setDelta_time(const ros::Duration &value);
};

#endif /* ODOMETRY_H_ */

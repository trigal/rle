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

#include "MeasurementModel.h"

/**
 * No measurement model equations are applied in this case for measuring the particle since
 * the visual odometry already returns the measured particle state.
 *
 * This method was written for future developing, or for applying a different measurement model
 *
 *
 * @param p_state 12x1 VectorXd
 * @return measured_p_state 12x1 VectorXd
 */
State6DOF MeasurementModel::measurePose(State6DOF& p_state){

//    VectorXd measured_p_state = VectorXd::Zero(12);
//    measured_p_state = p_state;

    State6DOF tmp = p_state;

    return tmp;
}

void MeasurementModel::setMsg(const nav_msgs::Odometry &m)
{

        _msg = m;
        _msr_cov = Utils::getCovFromOdom(m);

        tf::StampedTransform new_transform;
        new_transform.setOrigin(tf::Vector3(_msg.pose.pose.position.x, _msg.pose.pose.position.y, _msg.pose.pose.position.z));
        new_transform.setRotation(tf::Quaternion( _msg.pose.pose.orientation.x, _msg.pose.pose.orientation.y, _msg.pose.pose.orientation.z, _msg.pose.pose.orientation.w));

        tf::StampedTransform t1,t2;
        tf::Transform t3;
        tf::Vector3 tmp_translational_velocity;

        tf::StampedTransform fixed_transform;
        Eigen::AngleAxisd tmp_rotational_velocity;


        try
        {
            _listener->waitForTransform("visual_odometry_odom_x_forward","odom",ros::Time(0),ros::Duration(0.1));
            _listener->lookupTransform("visual_odometry_odom_x_forward","odom",ros::Time(0),fixed_transform);
//            _listener->lookupTransform("visual_odometry_odom_x_forward","odom",ros::Time::now(),fixed_transform);
//            _listener->lookupTwist("visual_odometry_car_frame", "visual_odometry_odom_x_forward", "visual_odometry_car_frame", tf::Point(0,0,0), "visual_odometry_car_frame", ros::Time(0), ros::Duration(.5), frame_speed); // TODO: wishful thinking

            _listener->lookupTransform("visual_odometry_odom_x_forward","visual_odometry_car_frame",_msg.header.stamp,t1);
            _listener->lookupTransform("visual_odometry_odom_x_forward","visual_odometry_car_frame",_msg.header.stamp - ros::Duration(0.1),t2); // TODO: Parametrize the derivation time
            t3 = t2.inverseTimes(t1);
            tmp_translational_velocity = t3.getOrigin() / 0.1;
            tmp_rotational_velocity = Eigen::Quaterniond(t3.getRotation().getW(),t3.getRotation().getX(),t3.getRotation().getY(),t3.getRotation().getZ());
            tmp_rotational_velocity.angle() /= 0.1;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            _msg_valid = false;
            return;
        }

        if(_first_run)
        {
            _old_transform = new_transform;
            _old_msg = _msg;
            _first_run = false;
            _msg_valid = false;
            return;
        }
        _msg_valid = true;

        tf::Transform delta_transform;

        delta_transform.setOrigin(fixed_transform * _old_transform.inverse() * new_transform.getOrigin());
        delta_transform.setBasis(_old_transform.inverse().getBasis() * new_transform.getBasis());
        tf::Quaternion tmp_q(delta_transform.getRotation().getZ(),-delta_transform.getRotation().getX(),-delta_transform.getRotation().getY(),delta_transform.getRotation().getW()); // TODO: CHECK THIS MAGIC OUT
        delta_transform.setRotation(tmp_q);

        _measure = State6DOF();
        _measure.setPose(Eigen::Vector3d(delta_transform.getOrigin().getX(),delta_transform.getOrigin().getY(),(delta_transform.getOrigin().getZ())));
        _measure.setRotation(Eigen::AngleAxisd(Eigen::Quaterniond(delta_transform.getRotation().getW(),delta_transform.getRotation().getX(),delta_transform.getRotation().getY(),delta_transform.getRotation().getZ())));
        _measure.setTranslationalVelocity(Eigen::Vector3d(tmp_translational_velocity.getX(),tmp_translational_velocity.getY(),tmp_translational_velocity.getZ()));
        _measure.setRotationalVelocity(tmp_rotational_velocity);

        _old_transform = new_transform;
        _old_msg = _msg;
}

MatrixXd MeasurementModel::measurementJacobian(State6DOF& p_state_predicted){
    /**
     * H_t:
     *
     *  | 1 0 0 0 0 0  0 0 0 0 0 0 |
     *  | 0 1 0 0 0 0  0 0 0 0 0 0 |
     *  | 0 0 1 0 0 0  0 0 0 0 0 0 |
     *  | 0 0 0 1 0 0  0 0 0 0 0 0 |
     *  | 0 0 0 0 1 0  0 0 0 0 0 0 |
     *  | 0 0 0 0 0 1  0 0 0 0 0 0 |
     *
     * 	| 0 0 0 0 0 0  1 0 0 0 0 0 |
     *  | 0 0 0 0 0 0  0 1 0 0 0 0 |
     *  | 0 0 0 0 0 0  0 0 1 0 0 0 |
     *  | 0 0 0 0 0 0  0 0 0 1 0 0 |
     *  | 0 0 0 0 0 0  0 0 0 0 1 0 |
     *  | 0 0 0 0 0 0  0 0 0 0 0 1 |
     */

    // Create an identity 12x12 matrix
    MatrixXd H_t = MatrixXd::Identity(12,12);

    return H_t;
}

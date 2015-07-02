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

void MeasurementModel::setMsg(const nav_msgs::Odometry &odometryMessage)
{
    ROS_DEBUG_STREAM("> Entering MeasurementModel::setMsg");

        /*  m is the OdometryCallback value (const nav_msgs::Odometry& msg)
         *  here we store the DELTA pose and the VELOCITY in the
         *  _measure Class Parameter
         *
         *                 pose           rotation
         *             trans_velocity   rot_velocity
         *
         *  following #416, i'm not using anymore the provided covariance matrix
         *
         *  task #417
         *  _msg_valid was used to handle LIBVISO2 gaps with the pre-decoupling version.
         *  With the decoupled version, the particles are updated independently from the
         *  odometry measures, only used to update de speeds.
         *  So, msg_valid is *not used* anymore.
         */

        _msg = odometryMessage;
        //_msg_valid = true;  //used in PARTICLE.cpp

//        std::cout << "=================================== " << std::endl << m << std::endl << Utils::getCovFromOdom(m) << std::endl;

//        _msr_cov = Eigen::MatrixXd::Identity(12,12) * 1.2; //   Utils::getCovFromOdom(m);

        /// this fixes #416
        //_msr_cov = Utils::getCovFromOdom(m);

        tf::StampedTransform new_transform;
        new_transform.setOrigin(tf::Vector3(_msg.pose.pose.position.x, _msg.pose.pose.position.y, _msg.pose.pose.position.z));
        new_transform.setRotation(tf::Quaternion( _msg.pose.pose.orientation.x, _msg.pose.pose.orientation.y, _msg.pose.pose.orientation.z, _msg.pose.pose.orientation.w));
        new_transform.stamp_=odometryMessage.header.stamp; //adding this little detail ...// Augusto sun.night

        if(measurementModelFirstRunNotExecuted)
        {
            ROS_WARN_STREAM("Setting message, but first run detected");
            _old_transform = new_transform;
            _old_msg = _msg;
            measurementModelFirstRunNotExecuted = false;
            ROS_WARN_STREAM("Setting measurementModelFirstRunNotExecuted = " << measurementModelFirstRunNotExecuted);

            //_msg_valid = false;                 //this could be false in different ways, now is useless since we use measurementModelFirstRun. related to #417
            ROS_DEBUG_STREAM("< Exiting MeasurementModel::setMsg cause First_Run");
            return;
        }

        ROS_INFO_STREAM_ONCE("Here we go.. the deltas will now be calculated. "<< " THIS MESSAGE WILL NOT APPEAR ANYMORE");
        ROS_INFO_STREAM_ONCE("Old transform " << _old_transform.stamp_ << " THIS MESSAGE WILL NOT APPEAR ANYMORE");
        ROS_INFO_STREAM_ONCE("New transform " << new_transform.stamp_  << " THIS MESSAGE WILL NOT APPEAR ANYMORE");

        tf::StampedTransform t1,t2;
        tf::Transform t3;
        tf::Vector3 tmp_translational_velocity;
        t1.setIdentity();
        t2.setIdentity();
        t3.setIdentity();
        tmp_translational_velocity.setZero();

        Eigen::AngleAxisd tmp_rotational_velocity;
        tmp_rotational_velocity.Identity();


        delta_time = new_transform.stamp_-_old_transform.stamp_;// Augusto sun.night
        ROS_DEBUG_STREAM("void MeasurementModel::setMsg > calculated delta time:" << delta_time.toSec());
        //ROS_ASSERT(delta_time==LayoutManager::delta_t); //TODO: check, if not assert [OK] we may use the LayoutManager::delta_t

        // try
        // {
        //         // One time for all, don't used anymore, fixed with LayoutManager.cpp
        //         //_listener->waitForTransform("visual_odometry_odom_x_forward","odom",_old_transform.stamp_,ros::Duration(1.1)); //this is fixed, do not lookup for it every time
        //         //_listener->lookupTransform("visual_odometry_odom_x_forward","odom",ros::Time(0),fixed_transform);
        //         //_listener->lookupTransform("visual_odometry_odom_x_forward","odom",_old_transform.stamp_ ,fixed_transform);
        //
        //         // TF LOOKUPTWIST VERSION
        //         // Lookup the twist of the tracking_frame with respect to the observation frame in the reference_frame using the reference point.
        //         // _listener->lookupTwist("visual_odometry_car_frame", "visual_odometry_odom_x_forward", "visual_odometry_car_frame", tf::Point(0,0,0), "visual_odometry_car_frame", ros::Time(0), ros::Duration(0.1), frameSpeed);
        //         _listener->lookupTwist("visual_odometry_car_frame", "visual_odometry_odom_x_forward",_msg.header.stamp,ros::Duration(0.05),frameSpeed);// Augusto sun.night
        //
        //         //_listener->waitForTransform("visual_odometry_odom_x_forward","visual_odometry_car_frame",_msg.header.stamp,ros::Duration(0.1));       // WAIT
        //         //_listener->lookupTransform("visual_odometry_odom_x_forward","visual_odometry_car_frame",_msg.header.stamp,t1);                        // published by LIBVISO, T1
        //         //_listener->lookupTransform("visual_odometry_odom_x_forward","visual_odometry_car_frame",_msg.header.stamp - ros::Duration(0.1),t2);   // old transform to evaluate speed, T2, fixed time interval
        //         //_listener->lookupTransform("visual_odometry_odom_x_forward","visual_odometry_car_frame",_old_transform.stamp_,t2);                    // use the previous transform timestamp
        //
        //         // Calculate the DELTA displacements
        //         // t3 = t2.inverseTimes(t1);
        //
        //         // From the delta displacements, calculate SPEEDs
        //         //tmp_translational_velocity = t3.getOrigin() / differentiate.toSec(); //tmp_translational_velocity = t3.getOrigin() / 0.1;
        //         //tmp_rotational_velocity = Eigen::Quaterniond(t3.getRotation().getW(),t3.getRotation().getX(),t3.getRotation().getY(),t3.getRotation().getZ());
        //         //tmp_rotational_velocity.angle() /= differentiate.toSec();
        //
        //
        //         // std::cout << tmp_translational_velocity.getX() << "\t"<< tmp_translational_velocity.getY() << "\t"<< tmp_translational_velocity.getZ()  << std::endl;
        // }
        // catch (tf::TransformException &ex)
        // {
        //         //_old_transform = new_transform; //mica giusto ma vabbé è per fare test delle velocità calcolate in forma AXEL+TRANSFORMs+LOOKUP
        //
        //         ROS_ERROR_STREAM("MeasurementModel.cpp setMsg says: %s" << ex.what());
        //         _msg_valid = false;
        //         return;
        // }


        tf::Transform delta_transform;

        delta_transform.setOrigin(fixed_transform * _old_transform.inverse() * new_transform.getOrigin());
        delta_transform.setBasis(_old_transform.inverse().getBasis() * new_transform.getBasis());
        tf::Quaternion tmp_q(delta_transform.getRotation().getZ(),-delta_transform.getRotation().getX(),-delta_transform.getRotation().getY(),delta_transform.getRotation().getW()); // TODO: CHECK THIS MAGIC OUT
        tmp_q.normalize();
        delta_transform.setRotation(tmp_q);

        // 1 and 2 are the same
        //ROS_INFO_STREAM("VEL_AXEL: " << tmp_translational_velocity.getX() << "\t"<< tmp_translational_velocity.getY() << "\t"<< tmp_translational_velocity.getZ());
        //ROS_INFO_STREAM("VEL_DELT: " << delta_transform.getOrigin().getX()/delta_time.toSec() << "\t"<< delta_transform.getOrigin().getY()/delta_time.toSec() << "\t"<< delta_transform.getOrigin().getZ()/delta_time.toSec());
        //ROS_INFO_STREAM("LIN_LUTW: " << frameSpeed.linear.x << "\t"<< frameSpeed.linear.y << "\t"<< frameSpeed.linear.z );
        //ROS_INFO_STREAM("ANG_LUTW: " << frameSpeed.angular.x << "\t"<< frameSpeed.angular.y << "\t"<< frameSpeed.angular.z );

        //ROS_INFO_STREAM( "T3:" << t3.getOrigin().getX() << "\t"<< t3.getOrigin().getY() << "\t"<< t3.getOrigin().getZ());

        // pose wrt last position, is a DELTA, so is mostly moving forward! ** PROVED **
        _measure_Delta = State6DOF();
        _measure_Delta.setPose(Eigen::Vector3d(delta_transform.getOrigin().getX(),delta_transform.getOrigin().getY(),(delta_transform.getOrigin().getZ())));
        _measure_Delta.setRotation(Eigen::AngleAxisd(Eigen::Quaterniond(delta_transform.getRotation().getW(),delta_transform.getRotation().getX(),delta_transform.getRotation().getY(),delta_transform.getRotation().getZ())));

        // OLD AXEL BEHAVIOR
        //_measure.setTranslationalVelocity(Eigen::Vector3d(tmp_translational_velocity.getX(),tmp_translational_velocity.getY(),tmp_translational_velocity.getZ()));
        //_measure.setRotationalVelocity(tmp_rotational_velocity);

        // NEW BEHAVIOR
        _measure_Delta.setTranslationalVelocity(Eigen::Vector3d(delta_transform.getOrigin().getX()/delta_time.toSec() ,delta_transform.getOrigin().getY()/delta_time.toSec() ,(delta_transform.getOrigin().getZ()/delta_time.toSec() )));
        tmp_rotational_velocity=Eigen::AngleAxisd(Eigen::Quaterniond(delta_transform.getRotation().getW(),delta_transform.getRotation().getX(),delta_transform.getRotation().getY(),delta_transform.getRotation().getZ()));
        tmp_rotational_velocity.angle() /= delta_time.toSec();
        _measure_Delta.setRotationalVelocity(tmp_rotational_velocity);

        ROS_DEBUG_STREAM( "_measure (translationalVelocity m/s): " << _measure_Delta.getTranslationalVelocity()(0) << "\t"<< _measure_Delta.getTranslationalVelocity()(1) << "\t"<< _measure_Delta.getTranslationalVelocity()(2) );
        ROS_DEBUG_STREAM( "_measure (rotationalVelocity  rad/s): " << _measure_Delta.getRotationalVelocity().angle() << "\tAxis: "<< _measure_Delta.getRotationalVelocity().axis()(0) << "\t"<< _measure_Delta.getRotationalVelocity().axis()(1) << "\t"<< _measure_Delta.getRotationalVelocity().axis()(2) );

        // Twist comparison
        //tf::Vector3 v1(frameSpeed.angular.x,frameSpeed.angular.y,frameSpeed.angular.z);
        //tf::Vector3 v2(oldFrameSpeed.angular.x,oldFrameSpeed.angular.y,oldFrameSpeed.angular.z);
        //tf::Vector3 v3;
        //v3=v1-v2;
        //v3*=delta_time.toSec();
        //tf::Quaternion qv3;
        //qv3=tf::createQuaternionFromRPY(v3.x(),v3.y(),v3.z());
        //
        //ROS_ERROR_STREAM("TWIST:\t" << v3.x() << "\t"<< v3.y() << "\t"<< v3.z() << "\tDelta_t:\t" << delta_time);
        //
        //tf::Quaternion old=_old_transform.getRotation().normalized();
        //tf::Quaternion res=tf::createIdentityQuaternion();
        //res= qv3* old;
        //res.normalize();
        //
        //ROS_INFO_STREAM(new_transform.getRotation().getX() << "\t"<<new_transform.getRotation().getY() << "\t"<<new_transform.getRotation().getZ() << "\t"<<new_transform.getRotation().getW());
        //ROS_INFO_STREAM(res.getX() << "\t"<<res.getY() << "\t"<<res.getZ() << "\t"<<res.getW());

        oldFrameSpeed = frameSpeed;
        _old_transform = new_transform;
        _old_msg = _msg;

        ROS_DEBUG_STREAM("< Exiting MeasurementModel::setMsg");
}


tf::StampedTransform MeasurementModel::getFixed_transform() const
{
    return fixed_transform;
}

void MeasurementModel::setFixed_transform(const tf::StampedTransform &value)
{
    fixed_transform = value;
}


ros::Duration MeasurementModel::getDelta_time() const
{
    return delta_time;
}

void MeasurementModel::setDelta_time(const ros::Duration &value)
{
    delta_time = value;
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

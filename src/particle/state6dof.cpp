#include "state6dof.h"

State6DOF::State6DOF()
{
    _pose = Eigen::Vector3d::Zero();
    _rotation = Eigen::AngleAxisd::Identity();
    _translational_velocity = Eigen::Vector3d::Zero();
    _rotational_velocity = Eigen::AngleAxisd::Identity();
}

Eigen::MatrixXd State6DOF::subtract(State6DOF &to_be_subtracted)
{
    Eigen::Matrix<double,12,1> tmp_matrix = this->toVectorXd() - to_be_subtracted.toVectorXd();
    return tmp_matrix;
}


State6DOF State6DOF::addVectorXd(Eigen::VectorXd &to_be_added)
{
    State6DOF tmp;
    Eigen::VectorXd tmp_v = this->toVectorXd() + to_be_added;

    tmp._pose = tmp_v.block(0,0,3,1);
    if(tmp_v.block(3,0,3,1) == Eigen::VectorXd::Zero(3))
    {
        Eigen::VectorXd asd = Eigen::VectorXd::Zero(3);
        asd(0) = 1.0;
        tmp._rotation = Eigen::AngleAxisd(0,asd);
    }
    else
        tmp._rotation = Eigen::AngleAxisd(tmp_v.block(3,0,3,1).norm(),tmp_v.block(3,0,3,1).normalized());

    tmp._translational_velocity = tmp_v.block(6,0,3,1);
    if(tmp_v.block(9,0,3,1) == Eigen::VectorXd::Zero(3))
    {
        Eigen::VectorXd asd = Eigen::VectorXd::Zero(3);
        asd(0) = 1.0;
        tmp._rotational_velocity = Eigen::AngleAxisd(0,asd);
    }
    else
        tmp._rotational_velocity = Eigen::AngleAxisd(tmp_v.block(9,0,3,1).norm(),tmp_v.block(9,0,3,1).normalized());
    return tmp;
}

Eigen::VectorXd State6DOF::toVectorXd()
{
    Eigen::VectorXd tmp_vector(12);
    tmp_vector <<
            this->_pose,
            this->_rotation.axis() * this->_rotation.angle(),
            this->_translational_velocity,
            this->_rotational_velocity.axis() * this->_rotational_velocity.angle();
    return tmp_vector;
}

geometry_msgs::Pose State6DOF::toGeometryMsgPose()
{
    geometry_msgs::Pose tmp;
    tmp.position.x = this->_pose(0);
    tmp.position.y = this->_pose(1);
    tmp.position.z = this->_pose(2);

    Eigen::Quaterniond tmp_quat(this->_rotation);

    tmp.orientation.w = tmp_quat.w();
    tmp.orientation.x = tmp_quat.x();
    tmp.orientation.y = tmp_quat.y();
    tmp.orientation.z = tmp_quat.z();

    return tmp;
}

State6DOF::State6DOF(const nav_msgs::Odometry &odom_msg)
{
    this->_pose << odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z;
    this->_rotation = Eigen::AngleAxisd(Eigen::Quaterniond(odom_msg.pose.pose.orientation.w,odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,odom_msg.pose.pose.orientation.z));
    this->_translational_velocity << odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z;

    tf::Matrix3x3 tmp_rot;
    tmp_rot.setEulerYPR(odom_msg.twist.twist.angular.z, odom_msg.twist.twist.angular.y, odom_msg.twist.twist.angular.x);
    tf::Quaternion q;
    tmp_rot.getRotation(q);
    this->_rotational_velocity = Eigen::AngleAxisd(Eigen::Quaterniond(q.getW(),q.getX(), q.getY(), q.getZ()));
}

void State6DOF::addNoise(double position_offset, double orientation_offset, double linear_offset, double angular_offset)
{
   srand(time(0));
   this->_pose(0) += Utils::getNoise(position_offset);
   this->_pose(1) += Utils::getNoise(position_offset);
   this->_pose(2) += Utils::getNoise(position_offset);

   this->_rotation.angle() += Utils::getNoise(orientation_offset);

   this->_translational_velocity(0) += Utils::getNoise(linear_offset);
   this->_translational_velocity(1) += Utils::getNoise(linear_offset);
   this->_translational_velocity(2) += Utils::getNoise(linear_offset);

   this->_rotational_velocity.angle() += Utils::getNoise(angular_offset);
}
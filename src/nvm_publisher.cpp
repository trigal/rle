#include <boost/assign.hpp>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <Eigen/Eigen>
#include <fstream>

#define RECONSTRUCTION_FILE "/home/dario/workspace_ros/src/road_layout_estimation/src/nvm_files/reconstruction4-sorted-decimali_tagliati.txt"
#define CAMERAS_NUMBER 360


using namespace std;

vector<geometry_msgs::PoseStamped> pose_vec;
double FREQUENCY=1;
// mapper/odometry


/**
 * @brief getRotationMatrix
 * @param q
 * @return
 */
Eigen::Matrix3d getRotationMatrix(Eigen::Vector4d q) {
  Eigen::Matrix3d m;
  double qw, qx, qy, qz;
  double qq = q.norm();
  if(qq > 0) {
    qw = q[0] / qq;
    qx = q[1] / qq;
    qy = q[2] / qq;
    qz = q[3] / qq;
  }
  else {
    qw = 1;
    qx = 0;
    qy = 0;
    qz = 0;
  }
  m(0, 0) = qw * qw + qx * qx - qy * qy - qz * qz;
  m(0, 1) = 2 * qx * qy - 2 * qw * qz;
  m(0, 2) = 2 * qw * qy + 2 * qx * qz;
  m(1, 0) = 2 * qx * qy + 2 * qw * qz;
  m(1, 1) = qw * qw + qy * qy - qx * qx - qz * qz;
  m(1, 2) = 2 * qy * qz - 2 * qw * qx;
  m(2, 0) = 2 * qx * qz - 2 * qw * qy;
  m(2, 1) = 2 * qy * qz + 2 * qw * qx;
  m(2, 2) = qw * qw + qz * qz - qx * qx - qy * qy;
  return m;
}

/**
 * @brief getCameraCenterAfterRotation
 * @param c
 * @param r
 * @return
 */
Eigen::Vector3d getCameraCenterAfterRotation(const double c[3], Eigen::Matrix3d r) {
    Eigen::Vector3d t;
    for(unsigned short int i = 0; i < 3; ++i) {
        t[i] = -(r(i, 0) * c[0] + r(i, 1) * c[1] + r(i, 2) * c[2]);
    }
    return t;
}


/**
 * @brief loadNVM
 * @param path
 * @return
 */
int loadNVM(const char* path) {

    ROS_INFO("loadNVM started");

    std::ifstream reconstruction(path);

    if(reconstruction.is_open()) {
        ROS_INFO(".nvm file opened");
        ROS_INFO("Cameras found: %d", CAMERAS_NUMBER);

        std::string name[CAMERAS_NUMBER];
        double c[3];
        Eigen::Vector4d q, q1;
        Eigen::Matrix3d r;
        Eigen::Vector3d t;
        for(unsigned short int i = 0; i < CAMERAS_NUMBER; ++i) { //x ogni riga/camera

            reconstruction >> name[i];

            for(unsigned short int j = 0; j < 4; ++j) {
                reconstruction >> q[j];
            }
            for(unsigned short int j = 0; j < 3; ++j) {
                reconstruction >> c[j];
            }

            q1 = q;
            r = getRotationMatrix(q);
            t = getCameraCenterAfterRotation(c, r);

            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "robot_frame";
            pose.pose.position.x = t[0];
            pose.pose.position.y = t[1];
            pose.pose.position.z = t[2];
            pose.pose.orientation.w = q1[0];
            pose.pose.orientation.x = q1[1];
            pose.pose.orientation.y = q1[2];
            pose.pose.orientation.z = q1[3];

            pose_vec.push_back(pose);
        }
        return 0;

        reconstruction.close();
    }
    return false;
}


/**
 * @brief normalize_angle
 * @param z
 * @return
 */
double normalize_angle(double z)
{
  return atan2(sin(z),cos(z));
}

/**
 * @brief angle_diff
 * @param a
 * @param b
 * @return
 */
double angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize_angle(a);
  b = normalize_angle(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

/**
 * @brief getSpeed
 * @param pose_prec
 * @param pose_t
 * @return
 */
geometry_msgs::Twist getSpeed(const geometry_msgs::PoseStamped & pose_prec, const geometry_msgs::PoseStamped & pose_t){
    geometry_msgs::Twist speed;
    double rate = 1 / FREQUENCY;
    speed.linear.x = (pose_t.pose.position.x - pose_prec.pose.position.x) / rate;
    speed.linear.y = (pose_t.pose.position.y - pose_prec.pose.position.y) / rate;
    speed.linear.z = (pose_t.pose.position.z - pose_prec.pose.position.z) / rate;

    // Quaternion to RPY (step_prec)
    tf::Quaternion q1; tf::Quaternion q2;
    tf::quaternionMsgToTF(pose_prec.pose.orientation, q1);
    tf::Matrix3x3 m(q1);
    double roll_prec; double pitch_prec; double yaw_prec;
    m.getRPY(roll_prec, pitch_prec, yaw_prec);

    // Quaternion to RPY (step_t)
    tf::quaternionMsgToTF(pose_t.pose.orientation,q2);
    tf::Matrix3x3 m_t(q2);
    double roll_t; double pitch_t; double yaw_t;
    m_t.getRPY(roll_t, pitch_t, yaw_t);

    speed.angular.x = ( angle_diff(roll_t, roll_prec) ) / rate;
    speed.angular.y = ( angle_diff(pitch_t, pitch_prec) ) / rate;
    speed.angular.z = ( angle_diff(yaw_t, yaw_prec) ) / rate;

    return speed;
}






/***************************************************************************************************************/
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "nvm_publisher");
    ros::NodeHandle nh;

    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("mapper/pose", 1);
    ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>("mapper/odometry", 1);
    ros::Publisher p_array_publisher = nh.advertise<geometry_msgs::PoseArray>("mapper/pose_array", 1);

    int error_type = loadNVM(RECONSTRUCTION_FILE);

    if(error_type == 0) {
        ROS_INFO("Publishing Point Cloud and Pose");
    }

    ros::Rate loop_rate(FREQUENCY);

    geometry_msgs::PoseStamped old_pose = pose_vec[0];

    int i=0;
    while(ros::ok()) {

        // pose msg ---------------------------------------------------------------------------------------
        geometry_msgs::PoseStamped pose = pose_vec[i%CAMERAS_NUMBER];
        pose_publisher.publish(pose);

        // pose array msg ---------------------------------------------------------------------------------
        geometry_msgs::PoseArray p_array;
        p_array.poses.push_back(pose.pose);
        p_array.header.stamp = pose.header.stamp;
        p_array.header.frame_id = "robot_frame";
        p_array_publisher.publish(p_array);


        // odometry msg ----------------------------------------------------------------------------------
        nav_msgs::Odometry odom;
        odom.child_frame_id = "odom_frame";
        odom.header.frame_id = "robot_frame";
        odom.header.stamp = pose.header.stamp;
        odom.pose.pose = pose.pose;
        odom.twist.twist = getSpeed(old_pose, pose);
        double odom_err = 0.05*0.05;
        odom.twist.covariance =  boost::assign::list_of  (odom_err) (0)   (0)  (0)  (0)  (0)
                                                               (0)  (odom_err)  (0)  (0)  (0)  (0)
                                                               (0)   (0)  (odom_err) (0)  (0)  (0)
                                                               (0)   (0)   (0) (odom_err) (0)  (0)
                                                               (0)   (0)   (0)  (0) (odom_err) (0)
                                                               (0)   (0)   (0)  (0)  (0)  (odom_err) ;
        odom.pose.covariance =  boost::assign::list_of  (odom_err) (0)  ( 0)  (0)  (0)  (0)
                                                              (0) (odom_err)   (0)  (0)  (0)  (0)
                                                              (0)   (0)  (odom_err) (0)  (0)  (0)
                                                              (0)   (0)   (0) (odom_err) (0)  (0)
                                                              (0)   (0)   (0)  (0) (odom_err) (0)
                                                              (0)   (0)   (0)  (0)  (0)  (odom_err) ;
        odom_publisher.publish(odom);
        // -----------------------------------------------------------------------------------------------

        std::cout << "--------------------------------------------------------------------------------" << endl;
        std::cout << "[ Sent msg " << i << "]:" << std::endl;
        std::cout << " Position:" << std::endl;
        std::cout << "  x: " << odom.pose.pose.position.x << std::endl;
        std::cout << "  y: " << odom.pose.pose.position.y << std::endl;
        std::cout << "  z: " << odom.pose.pose.position.z << std::endl;
        std::cout << " Orientation quaternion: " << std::endl;
        std::cout << "  w: " << odom.pose.pose.orientation.w << std::endl;
        std::cout << "  x: " << odom.pose.pose.orientation.x << std::endl;
        std::cout << "  y: " << odom.pose.pose.orientation.y << std::endl;
        std::cout << "  z: " << odom.pose.pose.orientation.z << std::endl;
        std::cout << " Linear speed: " << std::endl;
        std::cout << "  x: " << odom.twist.twist.linear.x << std::endl;
        std::cout << "  y: " << odom.twist.twist.linear.y << std::endl;
        std::cout << "  z: " << odom.twist.twist.linear.z << std::endl;
        std::cout << " Angular speed: " << std::endl;
        std::cout << "  x: " << odom.twist.twist.angular.x << std::endl;
        std::cout << "  y: " << odom.twist.twist.angular.y << std::endl;
        std::cout << "  z: " << odom.twist.twist.angular.z << std::endl;
        std::cout << std::endl;



        // aggiorno i valori
        old_pose = pose;
        i=i+1;
        loop_rate.sleep();
    }
    return 0;
}

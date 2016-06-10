/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Authors:                                                              *
 *              Augusto Luis Ballardini - ballardini@disco.unimib.it       *
 *              Axel         Furlan     - furlan@disco.unimib.it           *
 *              Dario        Limongi    - dario.limongi@gmail.com          *
 *                                                                         *
 ***************************************************************************/

#ifndef LAYOUTMANAGER_H_
#define LAYOUTMANAGER_H_


#include "Utils.h"

#include "geometry_msgs/PoseArray.h"
#include "MeasurementModel.h"
#include "nav_msgs/Odometry.h"
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/package.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_listener.h>
//#include <tf_conversions/tf_eigen.h>
#include "visualization_msgs/Marker.h"
#include <visualization_msgs/MarkerArray.h>

// ROS CORE STUFF
#include <ros/console.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include "eigenmultivariatenormal.hpp"

// BOOST
#include <boost/math/distributions/normal.hpp>
#include <boost/random/uniform_real.hpp>

#include <road_layout_estimation/road_layout_estimationConfig.h>
#include <road_layout_estimation/getAllParticlesLatLon.h>

// OPENSTREETMAPS MODULE
#include "ira_open_street_map/get_closest_way_distance_utm.h"
#include "ira_open_street_map/latlon_2_xy.h"
#include "ira_open_street_map/snap_particle_xy.h"
#include "ira_open_street_map/xy_2_latlon.h"
#include "ira_open_street_map/getHighwayInfo.h"
#include "ira_open_street_map/getDistanceFromLaneCenter.h"
#include "ira_open_street_map/oneway.h"

// COMPONENTS
#include "particle/LayoutComponent.h"
#include "particle/LayoutComponent_Building.h"
#include "particle/LayoutComponent_RoadLane.h"
#include "particle/LayoutComponent_RoadState.h"
#include "particle/LayoutComponent_OSMDistance.h"

#include "particle/Particle.h"
#include "road_lane_detection/road_lane_array.h"
#include "road_lane_detection/road_lane.h"


//#include "road_layout_estimation/msg_roadState.h"  //ROADSTATECOMPONENT FAKE
#include "road_layout_estimation/msg_lines.h"
#include "road_layout_estimation/msg_debugInformation.h"

#include <building_detection/FacadesList.h>

#include <iomanip>
#include <iostream>
#include <vector>
#include <numeric>
#include <functional>
#include <math.h>
#include <fstream>
#include <limits>



using boost::math::normal;
using namespace Eigen;
using Eigen::MatrixXd;
using std::vector;

/**
 *----------------------------------------------------------------------------------------------
 * SPECIFICHE:
 * - pose particle -> filtro EKF
 *   pose componenti -> filtro particelle
 *
                 // - score -> confronto tra predict ed eq. di misura
                 // - weight -> quanto pesa nella somma di tutte le componenti
 *
 *
 * - CHECK HAS MOVED: norm(pose_t - pose_t-1) < threshold
 *
 *  SCORE = SCORE_MOTION_MODEL(kalman gain) + sum(SCORE_COMPONENTS)
 *
 *  EQUAZIONE DI MISURA:
 *      mi arriva la misura dal tracker (visual odometry), vettore 6 elementi (6DoF pose).
 *      L'equazione di misura sarà una matrice (vedi foglio). La derivata è paro paro
 *
 *--------------------------------------------------------------------------------------------------------
 *  CALCOLO V_t+1
 *
 *  PREDICTION:
 *      considero sempre l'equazione v t+1 = v t + R
 *
 *  UPDATE:
 *  caso 1: non mi arrivano dati da visual odometry
 *          l'accelerazione è data dalla costante di smorzamento (vedi formula su foglio)
 *  caso 2: nel caso mi arrivino dati, faccio l'update
 *
 *
 *----------------------------------------------------------------------------------------------
 *
 * DA FARE:
 * - trovare nuovo motion model
 * - implementare particle filter
 *----------------------------------------------------------------------------------------------
 */


/**
 * @brief The LayoutManager class
 * Implementation of the current layout estimator
 */
class LayoutManager
{
public:

    ///DEBUG!!!
    tf::Stamped<tf::Pose> GPS_RTK_LOCAL_POSE;
    pcl::PointCloud<pcl::PointXYZ>::Ptr facades_cloud_;
    ros::Publisher facades_pub;

    MeasurementModel* measurement_model; ///< our Measurment Model created in the CLASS CONSTRUCTOR
    MotionModel default_mtn_model;       ///< default motion model applied to new particles TODO: allineare a measurement_model
    static double deltaOdomTime;         ///< time between current message and last arrived message (static should be unnecessary here)
    static double deltaTimerTime;

    // Publishers
    ros::Publisher array_pub;
    ros::Publisher gps_pub;
    ros::Publisher street_publisher;
    ros::Publisher particle_publisher;
    ros::Publisher diff_publisher;
    ros::Publisher marker_pub;
    ros::Publisher marker_pub2;
    ros::Publisher publisher_marker_array;
    ros::Publisher publisher_marker_array_distances;
    ros::Publisher publisher_marker_array_angles;
    ros::Publisher publisher_z_snapped;
    ros::Publisher publisher_z_particle;
    ros::Publisher publisher_GT_RTK;

    ros::Publisher publisher_average_pose;
    ros::Publisher publisher_debugInformation;

    // Subscriber
    ros::Subscriber odometry_sub;
    ros::Subscriber road_lane_sub;
    ros::Subscriber roadState_sub;
    ros::Subscriber buildings_sub;

    // Services from OpenStreetMap package
    ros::ServiceClient service_client;
    ros::ServiceClient latlon_2_xy_client;
    ros::ServiceClient xy_2_latlon_client;
    ros::ServiceClient snap_particle_xy_client;
    ros::ServiceClient get_closest_way_distance_utm_client;
    ros::ServiceClient getHighwayInfo_client;   ///< this service is used during the initialization phase inside reconfigureCallBack and RoadStateCallback (just because I delete/create this component all the times)
    ros::ServiceClient getDistanceFromLaneCenter_client;
    ros::ServiceClient oneWay_client;

    // Services from this node
    ros::ServiceServer server_getAllParticlesLatLon;

    int num_particles;

    static int odometryMessageCounter;          ///< stores the current layout_manager step

    //WARNING: this is a copy!?
    ros::NodeHandle node_handle;                ///< Default ROS - LayoutManager handler

    // Dynamic reconfigure
    dynamic_reconfigure::Server<road_layout_estimation::road_layout_estimationConfig> dynamicReconfigureServer;
    dynamic_reconfigure::Server<road_layout_estimation::road_layout_estimationConfig>::CallbackType dynamicReconfigureCallback;

    geometry_msgs::PoseArray buildPoseArrayMsg(std::vector<shared_ptr<Particle> > &particles);

    ///
    /// \brief layoutEstimation
    /// \param timerEvent
    /// \return particle set al tempo t
    /// Genera una stima del layout al tempo t a partire dal currentLayout
    ///
    void layoutEstimation(const ros::TimerEvent &timerEvent);

    ///
    /// \brief odometryCallback
    /// \param visualOdometryMsg
    /// (was the temp odometryCallback2);
    ///
    void odometryCallback(const nav_msgs::Odometry& visualOdometryMsg);

    ///
    /// \brief reconfigureCallback
    /// \param config
    /// \param level
    ///
    void reconfigureCallback(road_layout_estimation::road_layout_estimationConfig &currentConfiguration, uint32_t level);

    ///
    /// \brief roadLaneCallback
    /// \param msg
    ///
    void roadLaneCallback(const road_layout_estimation::msg_lines &msg_lines);

    void roadStateCallback(const road_layout_estimation::msg_lines& msg_lines);

    void buildingsCallback(const building_detection::FacadesList& facades);
    // getters & setters ----------------------------------------------------------------------------
//    MeasurementModel getVisualOdometry(){ return odometry; }
//    void setOdometry(MeasurementModel* v_odom){ odometry = v_odom; }


    // costructor & destructor ----------------------------------------------------------------------
    LayoutManager(ros::NodeHandle& n, std::string& topic, string &bagfile, double timerInterval, ros::console::Level loggingLevel);
    LayoutManager(const LayoutManager &other);

    LayoutManager& operator=(const LayoutManager&);

    ~LayoutManager()
    {
        ROS_INFO_STREAM("RLE is stopping ...");
        //current_layout.clear();
        current_layout_shared.clear();
//        stat_out_file.close();
        LIBVISO_out_file.close();
        RLE_out_file.close();
        RTK_GPS_out_file.close();
        delete measurement_model;
        node_handle.shutdown();
    }


    void normalizeParticleSet();
    void publishMarkerArray(double normalizationFactor);
    void publishMarkerArrayDistances(int id, double x1, double y1, double x2, double y2, double z);
    void publishZParticle(int id, double x1, double y1, double x2, double y2, double z);
    void publishZSnapped(int id, double x1, double y1, double x2, double y2, double z);
    void publish_initial_markers(double cov1, double cov2, geometry_msgs::Point point);


    // Timers
    ros::Timer RLE_timer_loop;                  ///< main loop timer
    void rleStart();
    void rleStop();

    ///
    /// \brief getAllParticlesLatLonService
    /// \param req
    /// \param resp
    /// \return
    ///
    /// Service Callback
    ///
    bool getAllParticlesLatLonService(road_layout_estimation::getAllParticlesLatLon::Request &req, road_layout_estimation::getAllParticlesLatLon::Response &resp);

    ROS_DEPRECATED tf::Stamped<tf::Pose> toGlobalFrame(Vector3d p_state);

    double getCurrent_layoutScore() const;
    void setCurrent_layoutScore(double value);

    // shared version of current layout
    ROS_DEPRECATED vector<shared_ptr<Particle> > getCurrent_layout_shared() const;
    void setCurrent_layout_shared(const vector<shared_ptr<Particle> > &value);

    // old version of curren layout
    //ROS_DEPRECATED vector<Particle> getCurrentLayout();
    //ROS_DEPRECATED void setCurrentLayout(vector<Particle>& p_set);

private:

    ///Spostato qui per testing / visualizzazione
    shared_ptr<Particle> bestParticle;

    /// Store here the configuration read with reconfigureCallBack, last configuration read.
    road_layout_estimation::road_layout_estimationConfig currentLayoutManagerConfiguration;

    tf::TransformListener tf_listener;
    boost::mt19937 rng;                                     ///< The uniform pseudo-random algorithm
    ROS_DEPRECATED double street_distribution_sigma;                       ///< Street gaussian distribution sigma
    ROS_DEPRECATED double angle_distribution_sigma;                        ///< Angle difference gaussian distribution sigma
    ROS_DEPRECATED double street_distribution_alpha;                      ///< Tells how does street pdf weight on score calculation
    ROS_DEPRECATED double angle_distribution_alpha;                       ///< Tells how does angle pdf weight on score calculation
    double roadState_distribution_alpha;                    ///< Tells how does roadStateComponents weight on the score calculation
    int    resampling_interval;                             ///< The resampling interval of the main Particle Filter

    static bool openstreetmap_enabled;                      ///< check this flag if we want to initialize particle-set with GPS and associate OSM score
    static bool layoutManagerFirstRun;                      ///< flag used for initiliazing particle-set with gps
    static bool first_msg;                                  ///< flag used for init particle-set
    nav_msgs::Odometry visualOdometryOldMsg;                ///< used for delta_t calculation

    bool new_detections;                                    ///< indicates detectors found new detections (not used)
    //ROS_DEPRECATED vector<Particle> current_layout;         ///< stores the current layout, changing Particle to Particle* to fix #529
    vector< shared_ptr<Particle> > current_layout_shared;   ///< stores the current layout, changing Particle to Particle* to fix #529
    double current_layoutScore;
    long resampling_count;

    MatrixXd particle_poses_statistics;                     ///< To calculate the statistics of the particle set for evaluation purposes (localization confidence)

    // Output files
    ofstream stat_out_file;                                 ///< output file, for framework evaluation
    ofstream LIBVISO_out_file;                              ///< output file, for framework evaluation
    ofstream RLE_out_file;                                  ///< output file, for framework evaluation
    ofstream RTK_GPS_out_file;                              ///< output file, for framework evaluation

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::MarkerArray marker_array_distances;
    visualization_msgs::MarkerArray marker_array_angles;
    visualization_msgs::MarkerArray marker_z_snapped;
    visualization_msgs::MarkerArray marker_z_particle;
    visualization_msgs::MarkerArray marker_array_GT_RTK;

    string bagfile;                             ///< This is used to cache the KITTI_XX.bags initialization parameters rather than GPS

    // Loader for nodelets.
    //nodelet::Loader     *manager;               ///< Loader for the nodelets.

    bool start_with_gps_message; ///< select RLE mode, hard-coded KITTI initializations, or GPS message

    /**
     * @brief checkHasMoved
     * @return
     */
    bool checkHasMoved();

    /**
     * @brief componentsEstimation
     * STEP 1: SAMPLING (PREDICT COMPONENTS POSES)
     * STEP 2: PERTURBATE COMPONENT POSES
     * STEP 3: WEIGHT LAYOUT-COMPONENTS
     */
    void componentsEstimation();

    /**
     * @brief sampling
     * Sampling from the state transition p(x_t | u_t , x_t-1):
     * we propagate the particle and its components with the motion model
     * and generate a predicted particle-set
     */
    void sampling();

    void componentsPerturbation();

    void calculateLayoutComponentsWeight();

    /**
     * @brief resampling
     * @param particle-set predetto
     * @return particle-set con resampling
     *
     * Resampling sul particle-set predetto, utilizzando lo score delle particelle:
     * chi ha peso più alto è più probabile che venga preso [roulette-wheel]
     */
    void resampling();


    /**
     * @brief calculateScore
     * @param particle_itr
     *
     * Cardinalità unaria (Particella presa INDIVIDUALMENTE)
     *  1- Kalman gain sulla pose della particella
     *  2- Somma dei WEIGHT delle varie componenti della particella (ottenuti dal filtraggio a particelle)
     *
     * Cardinalità >= 2
     *  1- plausibilità di esistenza tra le varie componenti di stesso tipo (due building sovrapposti ecc.)
     *  2- plausibilità di esistenza tra componenti di diverso tipo (building sovrapposto a una macchina ecc.)
     *
     * Nessuna particella verrà eliminata durante il procedimento di calcolo dello score,
     * essa sarà mantenuta in vita nonostante lo score sia basso.
     * In questo modo si evita la possibilità di eliminare dal particle-set ipotesi plausibili che abbiano ricevuto
     * uno score di valore basso per motivi di natura diversa.
     */
    ROS_DEPRECATED void calculateScore(const shared_ptr<Particle>& particle_itr_shared);//(Particle *particle_itr);

    /**
     * @brief calculateGeometricScores
     * @param particle_itr
     * helper function, while the distance (metric+angular) aren't components.
     */
    void calculateGeometricScores(const shared_ptr<Particle>& particle_itr);//(Particle *particle_itr);

};

#endif /* LAYOUTMANAGER_H_ */

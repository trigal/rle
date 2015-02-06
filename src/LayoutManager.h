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

#ifndef LAYOUTMANAGER_H_
#define LAYOUTMANAGER_H_

#include "Utils.h"
#include "Odometry.h"
#include "particle/Particle.h"
#include <vector>
#include "nav_msgs/Odometry.h"
#include "road_lane_detection/road_lane.h"
#include "road_lane_detection/road_lane_array.h"
#include "particle/LayoutComponent_RoadLane.h"
#include <sensor_msgs/NavSatFix.h>
#include "geometry_msgs/PoseArray.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <dynamic_reconfigure/server.h>
#include <road_layout_estimation/road_layout_estimationConfig.h>
#include <tf/transform_listener.h>

using namespace Eigen;
using Eigen::MatrixXd;
using std::vector;

/**
 *----------------------------------------------------------------------------------------------
 * SPECIFICHE:
 * - pose particle -> filtro EKF
 * 	 pose componenti -> filtro particelle
 *
                 // - score -> confronto tra predict ed eq. di misura
                 // - weight -> quanto pesa nella somma di tutte le componenti
 *
 *
 * - CHECK HAS MOVED: norm(pose_t - pose_t-1) < threshold
 *
 *  SCORE = SCORE_MOTION_MODEL(kalman gain) + sum(SCORE_COMPONENTS)
 *
 *	EQUAZIONE DI MISURA:
 *		mi arriva la misura dal tracker (visual odometry), vettore 6 elementi (6DoF pose).
 *		L'equazione di misura sarà una matrice (vedi foglio). La derivata è paro paro
 *
 *--------------------------------------------------------------------------------------------------------
 *	CALCOLO V_t+1
 *
 *	PREDICTION:
 *		considero sempre l'equazione v t+1 = v t + R
 *
 *	UPDATE:
 *	caso 1: non mi arrivano dati da visual odometry
 *			l'accelerazione è data dalla costante di smorzamento (vedi formula su foglio)
 *	caso 2: nel caso mi arrivino dati, faccio l'update
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
 * Implementation of the current layout estimator
 */
class LayoutManager {
public:
    Odometry* odometry;     /// our Measurment Model
    static double delta_t;  /// time between current message and last arrived message

    // Publishers
    ros::Publisher array_pub;
    ros::Publisher gps_pub;

    // Subscriber
    ros::Subscriber odometry_sub;
    ros::Subscriber road_lane_sub;

    // Services from OpenStreetMap package
    ros::ServiceClient service_client;
    ros::ServiceClient latlon_2_xy_client;
    ros::ServiceClient xy_2_latlon_client;
    ros::ServiceClient snap_particle_xy_client;

    int num_particles;

    static int step;           /// stores the current layout_manager step
    MotionModel mtn_model;

    // Dynamic reconfigure
    dynamic_reconfigure::Server<road_layout_estimation::road_layout_estimationConfig> server;
    dynamic_reconfigure::Server<road_layout_estimation::road_layout_estimationConfig>::CallbackType f;
    ros::NodeHandle node_handle;

    geometry_msgs::PoseArray buildPoseArrayMsg(std::vector<Particle>& particles);

private:

    tf::TransformListener tf_listener;

    static bool first_run;  /// flag used for initiliazing particle-set with gps
    static bool first_msg;  /// flag used for init particle-set
    nav_msgs::Odometry old_msg; /// used for delta_t calculation

    bool new_detections;				/// indicates detectors found new detections (not used)
    vector<Particle> current_layout;	/// stores the current layout


    /**
     * @brief checkHasMoved
     * @return
     */
    bool checkHasMoved();

    /**
     * STEP 1: SAMPLING (PREDICT COMPONENTS POSES)
     * STEP 2: PERTURBATE COMPONENT POSES
     * STEP 3: WEIGHT LAYOUT-COMPONENTS
     */
    void componentsEstimation();

    /**
	 * Sampling from the state transition p(x_t | u_t , x_t-1):
	 * we propagate the particle and its components with the motion model
     * and genera//currentLayout.resize(0);te a predicted particle-set
	 */
	void sampling();

    void componentsPerturbation();

    void calculateLayoutComponentsWeight();

	/**
	 * Resampling sul particle-set predetto, utilizzando lo score delle particelle:
	 * chi ha peso più alto è più probabile che venga preso [roulette-wheel]
	 * @param particle-set predetto
	 * @return particle-set con resampling
	 */
	void resampling();


    /**
     * FORMULA CALCOLO SCORE
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
     *
     */
	void calculateScore();

public:

    /**
     * Genera una stima del layout al tempo t a partire dal currentLayout
     * @return particle set al tempo t
     */
    vector<Particle> layoutEstimation();

    /**
     * @brief odometryCallback
     * @param msg
     */
    void odometryCallback(const nav_msgs::Odometry& msg);

    /**
     * @brief reconfigureCallback
     * @param config
     * @param level
     */
    void reconfigureCallback(road_layout_estimation::road_layout_estimationConfig &config, uint32_t level);

    /**
     * @brief roadLaneCallback
     * @param msg
     */
    void roadLaneCallback(const road_lane_detection::road_lane_array& msg);

    // getters & setters ----------------------------------------------------------------------------
//    Odometry getVisualOdometry(){ return odometry; }
//    void setOdometry(Odometry* v_odom){ odometry = v_odom; }

    vector<Particle> getCurrentLayout(){ return current_layout; }
    void setCurrentLayout(vector<Particle>& p_set){ current_layout = p_set; }

    // costructor & destructor ----------------------------------------------------------------------
    LayoutManager(ros::NodeHandle& n, std::string& topic);

    ~LayoutManager(){
        current_layout.clear();
        delete odometry;
    }
	LayoutManager(const LayoutManager &other);
	LayoutManager& operator=(const LayoutManager&);
};

#endif /* LAYOUTMANAGER_H_ */

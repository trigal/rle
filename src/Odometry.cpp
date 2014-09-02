/*
 * Odometry.cpp
 *
 *  Created on: May 25, 2014
 *      Author: dario
 */

#include "Odometry.h"

/**
 * No measurement model equations are applied in this case for measuring the particle since
 * the visual odometry already returns the measured particle state
 * This method was written for future developing, or for applying a different measurement model
 * @param p_state
 * @return measured_p_state
 */
VectorXd Odometry::measurePose(VectorXd& p_state){
    VectorXd measured_p_state = VectorXd::Zero(12);
    measured_p_state = p_state.head(12);
    return measured_p_state;
}

MatrixXd Odometry::measurementJacobi(VectorXd& p_state_predicted){
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

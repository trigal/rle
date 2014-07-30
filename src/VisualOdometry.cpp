/*
 * VisualOdometry.cpp
 *
 *  Created on: May 25, 2014
 *      Author: dario
 */

#include "VisualOdometry.h"

/**
 * No measurement model equations are applied in this case for measuring the particle since
 * the visual odometry already returns the measured particle state
 * This method was written for future developing, or for applying a different measurement model
 * @param p_state
 * @return measured_p_state
 */
VectorXd VisualOdometry::measurePose(VectorXd& p_state){
	VectorXd measured_p_state = VectorXd::Zero(12);
	measured_p_state = p_state.head(12);
	return measured_p_state;
}

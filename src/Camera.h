/*
 * Camera.h
 *
 *  Created on: May 25, 2014
 *      Author: dario
 */

#ifndef CAMERA_H_
#define CAMERA_H_
#include <string>

class Camera {
private:
	int id_camera; /// Camera ID
//	template<typename CAMERA_DATA>
//	CAMERA_DATA camera_data; /// Camera's datas
public:

	/**
	 * Returns data from the camera
	 * @return
	 */
	template<typename CAMERA_DATA>
	CAMERA_DATA getCameraData();

	// constructor, destructor
	Camera() : id_camera(0) {};
	Camera(int id) : id_camera(id) {};
	virtual ~Camera();
};

#endif /* CAMERA_H_ */

/*
 * BaseFrame.h
 *
 *  Created on: Oct 31, 2018
 *      Author: sujiwo
 */

#ifndef _BASEFRAME_H_
#define _BASEFRAME_H_


#include <Eigen/Eigen>
#include <opencv2/core.hpp>

#include "CameraPinholeParams.h"
#include "utilities.h"


class BaseFrame
{
public:
	BaseFrame();
	virtual ~BaseFrame();

	const Pose& pose() const
	{ return mPose; }

	void setPose (const Eigen::Vector3d &p, const Eigen::Quaterniond &q)
	{ mPose = Pose::from_Pos_Quat(p, q); }

	void setPose (const Pose &p)
	{ mPose = p; }

	// Project to 2D
	Eigen::Vector2d project (const Eigen::Vector3d &pt3) const;

	void setCameraParam(const CameraPinholeParams *c)
	{ cameraParam = const_cast<CameraPinholeParams*>(c); }

	/*
	 * This matrix transforms points in World Coordinate to Frame-centric coordinate
	 */
	Eigen::Matrix4d externalParamMatrix4 () const;

	Eigen::Matrix<double,3,4> projectionMatrix () const;


protected:
	cv::Mat image;

	/*
	 * A word on pose: this variable stores frame's pose in world coordinate system,
	 * with X->left, Y->bottom, Z->front
	 */
	Pose mPose = Pose::Identity();

	CameraPinholeParams *cameraParam = nullptr;
};

#endif /* _BASEFRAME_H_ */

/*
 * NdtLocalizer.h
 *
 *  Created on: Sep 21, 2018
 *      Author: sujiwo
 */

/*
 * This class is a wrapper for NDT TKU
 */
#ifndef _NDTLOCALIZER_H_
#define _NDTLOCALIZER_H_

#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <ndt.h>

#include "utilities.h"


struct NdtLocalizerInitialConfig
{
	double
		init_x,
		init_y,
		init_z;
	double
		init_roll,
		init_pitch,
		init_yaw;
};


class NdtLocalizer
{
public:
	NdtLocalizer(const NdtLocalizerInitialConfig &initialConfig);

	void loadMap (const std::string &filename);

	void loadMap (pcl::PointCloud<pcl::PointXYZ>::ConstPtr mapcloud);

	void putEstimation (const Pose &pEst);

	Pose localize (pcl::PointCloud<pcl::PointXYZ>::ConstPtr scan);

	Pose solveFromEstimation (const Pose &poseEstimation, pcl::PointCloud<pcl::PointXYZ>::ConstPtr scan);

	virtual ~NdtLocalizer();

	Eigen::Vector3d getMinPoint() const
	{ return pointMin; }

	Eigen::Vector3d getMaxPoint() const
	{ return pointMax; }

	bool isPointInsideMap (const Eigen::Vector3d &pt) const;
	bool isPointInsideMap (const pcl::PointXYZ &pt) const;

protected:

	NDMapPtr ndMap;

	bool initialized = false;

	bool mapLoaded = false;

	Posture prev_pose, prev_pose2;

	Eigen::Vector3d
		pointMin=Eigen::Vector3d::Zero(),
		pointMax=Eigen::Vector3d::Zero();
};

#endif /* _NDTLOCALIZER_H_ */

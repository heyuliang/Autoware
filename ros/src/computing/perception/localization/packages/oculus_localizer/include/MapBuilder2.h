/*
 * MapBuilder2.h
 *
 *  Created on: Jul 26, 2018
 *      Author: sujiwo
 */

#ifndef MAPBUILDER2_H_
#define MAPBUILDER2_H_

#include <vector>
#include <string>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include "VMap.h"


class Viewer;


class InputFrame {
public:
	cv::Mat image;
	Eigen::Vector3d position = Eigen::Vector3d::Zero();
	Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
	bool positionIsValid = true;
	uint cameraId = 0;

	InputFrame() {}

	InputFrame (const cv::Mat &i, const Eigen::Vector3d &p, const Eigen::Quaterniond &o) :
		image(i),
		position(p),
		orientation(o)
	{}
};


class MapBuilder2
{

public:

	MapBuilder2 ();
	virtual ~MapBuilder2();

	void initialize (const InputFrame &f1, const InputFrame &f2);

	void track (const InputFrame &f);

	void build ();

	VMap* getMap()
	{ return cMap; }

	void addCameraParam (const CameraPinholeParams &c)
	{ cMap->addCameraParameter(c); }


protected:

	CameraPinholeParams cparams;

	VMap *cMap;

	cv::Mat mask;

	kfid currentAnchor;

	bool initialized = false;

	Viewer *imageView;
};

#endif /* MAPBUILDER2_H_ */

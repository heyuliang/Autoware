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
#include <functional>
#include "VMap.h"
#include "utilities.h"
#include "datasets/GenericDataset.h"


class Viewer;


class InputFrame {
public:
	cv::Mat image;
	Eigen::Vector3d position = Eigen::Vector3d::Zero();
	Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
	bool positionIsValid = true;
	uint cameraId = 0;
	kfid setKfId=std::numeric_limits<kfid>::max();
	ptime tm=boost::posix_time::not_a_date_time;

	InputFrame() {}

	InputFrame (const cv::Mat &i, const Eigen::Vector3d &p, const Eigen::Quaterniond &o, kfid _forceKfId=std::numeric_limits<kfid>::max()) :
		image(i),
		position(p),
		orientation(o),
		setKfId(_forceKfId)
	{}

	Pose getPose()
	{ return Pose::from_Pos_Quat(position, orientation); }
};


class MapBuilder2
{

public:

	MapBuilder2 ();
	virtual ~MapBuilder2();

	void initialize (const InputFrame &f1, const InputFrame &f2);

	void track (const InputFrame &f);

	void track2 (const InputFrame &f);

	void build ();

	VMap* getMap()
	{ return cMap; }

	void addCameraParam (const CameraPinholeParams &c)
	{ cMap->addCameraParameter(c); }

	inline kfid getCurrentKeyFrameId()
	{ return currentAnchor; }

	void resetMap();

	typedef std::function<void(const InputFrame&)> frameCallback;
	void registerFrameCallback (const frameCallback& f)
	{ fInCal = f; }

	void runFromDataset (GenericDataset *ds);

protected:

	CameraPinholeParams cparams;

	VMap *cMap;

	cv::Mat mask;

	kfid currentAnchor;

	bool initialized = false;

	GenericDataset *sourceDataset=NULL;
	frameCallback fInCal;

	InputFrame frame0;
};

#endif /* MAPBUILDER2_H_ */

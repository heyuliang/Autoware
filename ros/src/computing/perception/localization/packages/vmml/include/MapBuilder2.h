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

const double
	translationThrs = 1.0,	// meter
	rotationThrs = 0.04;	// == 2.5 degrees



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

	Pose getPose() const
	{ return Pose::from_Pos_Quat(position, orientation); }
};


class MapBuilder2
{

public:

	MapBuilder2 ();
	virtual ~MapBuilder2();

	void input (const InputFrame &f);

	void build ();

	VMap* getMap()
	{ return cMap; }

	void addCameraParam (const CameraPinholeParams &c)
	{ cMap->addCameraParameter(c); }

	inline kfid getCurrentKeyFrameId()
	{ return kfAnchor; }

	void resetMap();

	typedef std::function<void(const InputFrame&)> frameCallback;
	void registerFrameCallback (frameCallback& f)
	{ inputCallback = f; }

	void runFromDataset (GenericDataset *ds);

protected:

	CameraPinholeParams cparams;

	VMap *cMap;

	cv::Mat mask;

	kfid kfAnchor;
	InputFrame ifrAnchor;

	bool initialized = false;

	GenericDataset *sourceDataset=NULL;
	frameCallback inputCallback;

	InputFrame frame0;

private:
	static bool isNormalFrame (const InputFrame &f);

	void initialize (const InputFrame &f1, const InputFrame &f2);

	void track (const InputFrame &f);

};

#endif /* MAPBUILDER2_H_ */

/*
 * MeidaiBag.h
 *
 *  Created on: Aug 10, 2018
 *      Author: sujiwo
 */


#include <string>
#include <memory>
#include <vector>
#include <rosbag/bag.h>

#include "utilities.h"
#include "datasets/GenericDataset.h"
#include "datasets/RandomAccessBag.h"


#ifndef _MEIDAIBAG_H_
#define _MEIDAIBAG_H_


class PoseStamped : public Pose
{
protected:
	ros::Time timestamp;
};


class Trajectory : public vector<PoseStamped>
{
public:

	void push_back(const PoseStamped &);

	PoseStamped at(const ros::Time&) const;
	PoseStamped at(const double) const;

	PoseStamped interpolate (const double) const;
	PoseStamped interpolate (const ros::Time&) const;
};


class MeidaiDataItem : public GenericDataItem
{
};


class MeidaiBag : public GenericDataset
{
public:

	MeidaiBag(const std::string &filePath);
	virtual ~MeidaiBag();

	size_t size() const;

	CameraPinholeParams getCameraParameter();

	cv::Mat getMask();

	MeidaiDataItem& at(dataItemId i) const;

protected:
	static std::string dSetName;
	rosbag::Bag *bagfd;
	RandomAccessBag *cameraRawBag;
};

#endif /* _MEIDAIBAG_H_ */

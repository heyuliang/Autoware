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


struct PoseTimestamp : public Pose
{
	PoseTimestamp(const Pose &p)
	{
		m_matrix = p.matrix();
		timestamp = ros::Time(0);
	}

	static PoseTimestamp interpolate(
		const PoseTimestamp &p1,
		const PoseTimestamp &p2,
		const ros::Time &t);

	ros::Time timestamp;
};


class Trajectory : public vector<PoseTimestamp>
{
public:

	void push_back(const PoseTimestamp &);

	// Return nearest element of provided time
	PoseTimestamp at(const ros::Time&) const;
//	PoseTimestamp at(const double) const;

	// Interpolate value
//	PoseTimestamp interpolate (const double) const;
	PoseTimestamp interpolate (const ros::Time&) const;

private:
	uint32_t
	find_lower_bound(const ros::Time&) const;

	typedef vector<PoseTimestamp> Parent;
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


void createTrajectoryFromGnssBag (RandomAccessBag &bagsrc, Trajectory &trajectory, int plane_number=7);

#endif /* _MEIDAIBAG_H_ */

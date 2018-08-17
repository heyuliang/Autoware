/*
 * MeidaiBag.cpp
 *
 *  Created on: Aug 10, 2018
 *      Author: sujiwo
 */

#include <exception>

#include "MeidaiBag.h"


using namespace std;

string MeidaiBag::dSetName = "Oxford";


MeidaiBag::MeidaiBag(const string &path)
{
	bagfd = new rosbag::Bag(path);
	cameraRawBag = new RandomAccessBag(bagfd, "/camera1/image_raw");
}


MeidaiBag::~MeidaiBag()
{
	delete(cameraRawBag);
	delete(bagfd);
}


size_t
MeidaiBag::size() const
{
	return cameraRawBag->size();
}


CameraPinholeParams
MeidaiBag::getCameraParameter()
{
	throw exception("Not implemented");
}


void
Trajectory::push_back(const PoseTimestamp &pt)
{
	assert(pt.getStamp() > back().getStamp());
	return std::vector<PoseTimestamp>::push_back(pt);
}


PoseTimestamp
Trajectory::at(const ros::Time& t) const
{

}


PoseTimestamp
Trajectory::interpolate (const ros::Time& t) const
{

}

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

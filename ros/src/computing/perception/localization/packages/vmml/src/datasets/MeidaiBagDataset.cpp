/*
 * MeidaiBag.cpp
 *
 *  Created on: Aug 10, 2018
 *      Author: sujiwo
 */

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>

#include <datasets/MeidaiBagDataset.h>
#include <exception>
#include <algorithm>
#include <string>



using namespace std;
using namespace Eigen;

namespace bfs = boost::filesystem;

string MeidaiBagDataset::dSetName = "Nagoya University";


MeidaiBagDataset::MeidaiBagDataset(

	const string &path,
	double startTimeOffsetSecond,
	double mappingDurationSecond,
	const std::string &calibrationPath) :

		bagPath(path)
{
	bagfd = new rosbag::Bag(path);
	cameraRawBag = new RandomAccessBag(*bagfd, "/camera1/image_raw");
	loadCache();
}


MeidaiBagDataset::~MeidaiBagDataset()
{
	delete(cameraRawBag);
	delete(bagfd);
}


size_t
MeidaiBagDataset::size() const
{
	return cameraRawBag->size();
}


CameraPinholeParams
MeidaiBagDataset::getCameraParameter()
{
	throw runtime_error("Not implemented");
}


cv::Mat
MeidaiBagDataset::getMask()
{
	return cv::Mat();
}


MeidaiDataItem&
MeidaiBagDataset::at(dataItemId i) const
{
	// XXX: Stub
	throw runtime_error("Not implemented");
}


void
MeidaiBagDataset::loadCache()
{
	bfs::path bagCachePath = bagPath;
	bagCachePath += ".cache";
	bool isCacheValid = false;

	// Check if cache is valid
	if (bfs::exists(bagCachePath) and bfs::is_regular_file(bagCachePath)) {
		auto lastWriteTime = bfs::last_write_time(bagCachePath),
			lastBagModifyTime = bfs::last_write_time(bagPath);
		if (lastWriteTime >= lastBagModifyTime)
			isCacheValid = true;
	}

	if (isCacheValid) {
		doLoadCache(bagCachePath.string());
	}

	else {
		createCache();
		writeCache(bagCachePath.string());
	}
}


void
MeidaiBagDataset::doLoadCache(const string &path)
{
	fstream cacheFd;
	cacheFd.open(path.c_str(), fstream::in);
	if (!cacheFd.is_open())
		throw runtime_error(string("Unable to open cache file: ") + path);

	boost::archive::binary_iarchive cacheIArc (cacheFd);

	cacheIArc >> gnssTrack;
	cacheIArc >> ndtTrack;

	cacheFd.close();
}


void
MeidaiBagDataset::createCache()
{
	RandomAccessBag ramGnssBag (*bagfd, "/nmea_sentence");
	createTrajectoryFromGnssBag(ramGnssBag, gnssTrack);
}


void MeidaiBagDataset::writeCache(const string &path)
{
	fstream cacheFd;
	cacheFd.open(path.c_str(), fstream::out);
	if (!cacheFd.is_open())
		throw runtime_error(string("Unable to open cache file: ") + path);

	boost::archive::binary_oarchive cacheOArc (cacheFd);

	cacheOArc << gnssTrack;
	cacheOArc << ndtTrack;

	cacheFd.close();
}


void
Trajectory::push_back(const PoseTimestamp &pt)
{
	if (size()>0)
		assert(pt.timestamp >= back().timestamp);
	return Parent::push_back(pt);
}


uint32_t
Trajectory::find_lower_bound(const ros::Time &t) const
{
	auto it = std::lower_bound(begin(), end(), t,
		[](const PoseTimestamp &el, const ros::Time& tv)
			-> bool {return el.timestamp < tv;}
	);
	return it-begin();
}


PoseTimestamp
Trajectory::at(const ros::Time& t) const
{
	return Parent::at(find_lower_bound(t));
}


PoseTimestamp
Trajectory::interpolate (const ros::Time& t) const
{
	assert (t < (*end()).timestamp);

	uint32_t i0 = find_lower_bound(t),
		i1 = i0+1;
	return PoseTimestamp::interpolate(Parent::at(i0), Parent::at(i1), t);
}


void
MeidaiDataItem::init()
{
	bImageMsg = parent->cameraRawBag->at<sensor_msgs::Image>(pId);
}


/*
 * XXX: Stub
 */
Vector3d
MeidaiDataItem::getPosition() const
{
	return Vector3d::Zero();
}


/*
 * XXX: Stub
 */
Quaterniond
MeidaiDataItem::getOrientation() const
{
	return Quaterniond::Identity();
}


cv::Mat
MeidaiDataItem::getImage() const
{
	// XXX: CV Bridge
}


ptime
MeidaiDataItem::getTimestamp() const
{
	return bImageMsg->header.stamp.toBoost();
}

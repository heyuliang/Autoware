/*
 * MeidaiBag.cpp
 *
 *  Created on: Aug 10, 2018
 *      Author: sujiwo
 */

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/filesystem.hpp>

#include <exception>
#include <algorithm>
#include <string>
#include <ros/ros.h>
#include "datasets/MeidaiBagDataset.h"


using namespace std;
using namespace Eigen;

namespace bfs = boost::filesystem;

string MeidaiBagDataset::dSetName = "Nagoya University";

const string
	meidaiBagImageTopic = "/camera1/image_raw",
	meidaiBagGnssTopic  = "/nmea_sentence",
	meidaiBagVelodyne   = "/velodyne_packets";


MeidaiBagDataset::MeidaiBagDataset(

	const string &path,
	double startTimeOffsetSecond,
	double mappingDurationSecond,
	const std::string &calibrationPath,
	bool loadPositions) :

		bagPath(path)
{
	bagfd = new rosbag::Bag(path);
	cameraRawBag = RandomAccessBag::Ptr (new RandomAccessBag(*bagfd, meidaiBagImageTopic));
	gnssBag = RandomAccessBag::Ptr (new RandomAccessBag(*bagfd, meidaiBagGnssTopic));
	velodyneBag = RandomAccessBag::Ptr (new RandomAccessBag(*bagfd, meidaiBagVelodyne));

	if (loadPositions==true)
		loadCache();
}


MeidaiBagDataset::Ptr
MeidaiBagDataset::load (
		const string &path,
		double startTimeOffsetSecond,
		double mappingDurationSecond,
		const std::string &calibrationPath,
		bool loadPositions
)
{
	MeidaiBagDataset::Ptr nuDatasetMem(new MeidaiBagDataset(
		path,
		startTimeOffsetSecond,
		mappingDurationSecond,
		calibrationPath,
		loadPositions));
	return nuDatasetMem;
}


void
MeidaiBagDataset::loadPosition()
{
	loadCache();
}


MeidaiBagDataset::~MeidaiBagDataset()
{
	delete(bagfd);
}


size_t
MeidaiBagDataset::size() const
{
	return cameraRawBag->size();
}


CameraPinholeParams
MeidaiBagDataset::getCameraParameter()
const
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


GenericDataItem::ConstPtr
MeidaiBagDataset::get(dataItemId i)
const
{
	MeidaiDataItem::ConstPtr dp(new MeidaiDataItem(*this, i));
	return dp;
}


void
MeidaiBagDataset::setZoomRatio (float r)
{
	zoomRatio = r;
}


float
MeidaiBagDataset::getZoomRatio () const
{ return zoomRatio; }


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
MeidaiBagDataset::setLidarParameters (
	const std::string &pvelodyneCalibrationFile,
	const std::string &pmeidaiPCDMapFile,
	const TTransform &plidarToCameraTransform)
{
	lidarToCameraTransform = plidarToCameraTransform;
	pcdMapFilePath = pmeidaiPCDMapFile;
	velodyneCalibrationFilePath = pvelodyneCalibrationFile;
}


void
MeidaiBagDataset::forceCreateCache ()
{
	bfs::path bagCachePath = bagPath;
	bagCachePath += ".cache";

	gnssTrack.clear();
	ndtTrack.clear();
	cameraTrack.clear();
	createCache();
	writeCache(bagCachePath.string());
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
	cacheIArc >> cameraTrack;

	cacheFd.close();
}


void
MeidaiBagDataset::createCache()
{
	cout << "Creating GNSS Trajectory\n";
	createTrajectoryFromGnssBag(*gnssBag, gnssTrack);
	cout << "Creating NDT Trajectory\n";
	createTrajectoryFromNDT(*velodyneBag, ndtTrack, gnssTrack, velodyneCalibrationFilePath, pcdMapFilePath);

	cout << "Creating Camera Trajectory\n";
	// XXX: It is possible that camera recording may have started earlier than lidar's
	for (int i=0; i<cameraRawBag->size(); i++) {
		auto tm = cameraRawBag->timeAt(i);

		// in both cases; extrapolate
		if (tm <= ndtTrack[0].timestamp) {
			continue;
		}

		else if (tm >= ndtTrack.back().timestamp) {
			continue;
		}

		PoseTimestamp poseCur = ndtTrack.interpolate(tm);
		// XXX: Transform from lidar to camera
		cameraTrack.push_back(poseCur);
	}
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
	cacheOArc << cameraTrack;

	cacheFd.close();
}


GenericDataItem::ConstPtr
MeidaiBagDataset::atDurationSecond (const double second)
const
{
	uint32_t pos = cameraRawBag->getPositionAtDurationSecond(second);
	return get(pos);
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
	if ( t < front().timestamp or t > back().timestamp )
		throw out_of_range("Time out of range");

	auto it = std::lower_bound(begin(), end(), t,
		[](const PoseTimestamp &el, const ros::Time& tv)
			-> bool {return el.timestamp < tv;}
	);
	uint32_t x = it-begin();
	return x;
}


uint32_t
Trajectory::find_lower_bound(const ptime &t) const
{
	ros::Time tx = ros::Time::fromBoost(t);
	return find_lower_bound(tx);
}


PoseTimestamp
Trajectory::at(const ros::Time& t) const
{
	return Parent::at(find_lower_bound(t));
}


PoseTimestamp
Trajectory::at(const ptime &t) const
{

	return Parent::at(find_lower_bound(t));
}


PoseTimestamp
Trajectory::interpolate (const ros::Time& t) const
{
	assert (front().timestamp<=t and t < back().timestamp);

	uint32_t i0 = find_lower_bound(t),
		i1 = i0+1;
	return PoseTimestamp::interpolate(Parent::at(i0), Parent::at(i1), t);
}


PoseTimestamp
Trajectory::extrapolate (const ros::Time& t) const
{
	assert (t < front().timestamp or t > back().timestamp);
	// XXX: Unfinished
}


void
MeidaiDataItem::init()
{
	bImageMsg = parent.cameraRawBag->at<sensor_msgs::Image>(pId);
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
	auto imgPtr = cv_bridge::toCvShare(bImageMsg, sensor_msgs::image_encodings::BGR8);
	if (parent.zoomRatio==1.0)
		return imgPtr->image;
	else {
		cv::Mat imgrs;
		cv::resize(imgPtr->image, imgrs, cv::Size(), parent.zoomRatio, parent.zoomRatio, cv::INTER_CUBIC);
		return imgrs;
	}
}


ptime
MeidaiDataItem::getTimestamp() const
{
	return bImageMsg->header.stamp.toBoost();
}

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
	prepareBag();

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


MeidaiBagDataset::MeidaiBagDataset
(const MeidaiBagDataset &cp):
	bagPath(cp.bagPath),
	bagfd(cp.bagfd)
{}


MeidaiBagDataset::Ptr
MeidaiBagDataset::subset(const ros::Time &startTime, const ros::Duration &lengthInSecond) const
{
	MeidaiBagDataset *rsubset = new MeidaiBagDataset( *this );
	rsubset->isSubset_ = true;

	ros::Time tlast = startTime + lengthInSecond;
	rsubset->subsetBeginTime = startTime;
	rsubset->subsetEndTime = tlast;
	rsubset->prepareBag(startTime, tlast);

	// Load positions, if they are complete in this time range
	if (cameraTrack.empty())
		goto finish;

	if (cameraTrack.front().timestamp > startTime or cameraTrack.back().timestamp < tlast)
		goto finish;

	rsubset->cameraTrack = cameraTrack.subset(startTime, tlast);

finish:
	return MeidaiBagDataset::Ptr(rsubset);
}


void
MeidaiBagDataset::prepareBag (const ros::Time &beginTime, const ros::Time &stopTime)
{
	cameraRawBag = RandomAccessBag::Ptr(new RandomAccessBag(*bagfd, meidaiBagImageTopic, beginTime, stopTime));
	gnssBag = RandomAccessBag::Ptr(new RandomAccessBag(*bagfd, meidaiBagGnssTopic, beginTime, stopTime));
	velodyneBag = RandomAccessBag::Ptr(new RandomAccessBag(*bagfd, meidaiBagVelodyne, beginTime, stopTime));
}


MeidaiBagDataset::Ptr
MeidaiBagDataset::subset(const double startTimeOffsetSecond, const double endOffsetFromBeginning) const
{
	assert (endOffsetFromBeginning >= startTimeOffsetSecond);
	ros::Duration
		d0(startTimeOffsetSecond),
		d1(endOffsetFromBeginning-startTimeOffsetSecond);
	return subset(cameraRawBag->timeAt(0) + d0, d1);
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


size_t
MeidaiBagDataset::sizeAll() const
{
	RandomAccessBag bg(*bagfd, meidaiBagImageTopic);
	return bg.size();
}


CameraPinholeParams
MeidaiBagDataset::getCameraParameter()
const
{
	return cameraParams * zoomRatio;
}


void
MeidaiBagDataset::addCameraParameter(const CameraPinholeParams &c)
{ cameraParams = c; }


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
MeidaiBagDataset::forceCreateCache (bool resetSubset)
{
	bfs::path bagCachePath = bagPath;
	bagCachePath += ".cache";

	if (resetSubset==true) {
		isSubset_ = false;
		subsetBeginTime = ros::TIME_MIN;
		subsetEndTime = ros::TIME_MIN;
	}

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

	cacheIArc >> isSubset_;
	ptime tx;
	cacheIArc >> tx;
	subsetBeginTime = ros::Time::fromBoost(tx);
	cacheIArc >> tx;
	subsetEndTime = ros::Time::fromBoost(tx);

	if (isSubset_) {
		prepareBag(subsetBeginTime, subsetEndTime);
	}

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
		PoseTimestamp poseX;
		if (tm < ndtTrack[0].timestamp or tm>=ndtTrack.back().timestamp) {
			poseX = ndtTrack.extrapolate(tm);
		}
		else
			poseX = ndtTrack.interpolate(tm);
		// XXX: Check this value
		PoseTimestamp poseX1 = poseX * lidarToCameraTransform;
		cameraTrack.push_back(poseX1);

		cout << i+1 << " / " << cameraRawBag->size() << "  \r";
	}
}


void MeidaiBagDataset::writeCache(const string &path)
{
	fstream cacheFd;
	cacheFd.open(path.c_str(), fstream::out);
	if (!cacheFd.is_open())
		throw runtime_error(string("Unable to open cache file: ") + path);

	boost::archive::binary_oarchive cacheOArc (cacheFd);

	cacheOArc << isSubset_;
	ptime tx = subsetBeginTime.toBoost();
	cacheOArc << tx;
	tx = subsetEndTime.toBoost();
	cacheOArc << tx;

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

	uint32_t i0 = find_lower_bound(t)-1;
	uint32_t i1 = i0+1;
	return PoseTimestamp::interpolate(Parent::at(i0), Parent::at(i1), t);
}


PoseTimestamp
Trajectory::extrapolate (const ros::Time& t) const
{
	assert (t < front().timestamp or t > back().timestamp);

	double r;
	Vector3d xpos;
	Quaterniond xori;

	if (t > back().timestamp) {
		size_t sz = size();
		const PoseTimestamp
			&lastPose = back(),
			&lastPose_1 = Parent::at(sz-2);

		r = (t.toSec() - lastPose_1.timestamp.toSec()) /
			(lastPose.timestamp.toSec() - lastPose_1.timestamp.toSec());

		xpos = lastPose_1.position() + r*(lastPose.position() - lastPose_1.position());
		// XXX: Not true
		xori = lastPose.orientation();
	}

	else if (t < front().timestamp) {
		const PoseTimestamp
			&firstPose = front(),
			&firstPose_1 = Parent::at(1);

		r = (t.toSec() - firstPose_1.timestamp.toSec()) /
			(firstPose.timestamp.toSec() - firstPose_1.timestamp.toSec());
		xpos = firstPose_1.position() + r*(firstPose.position() - firstPose_1.position());
		// ??
		xori = firstPose.orientation();
	}

	return PoseTimestamp(xpos, xori, t);
}


Trajectory
Trajectory::subset(const ros::Time &start, const ros::Time &stop) const
{
	assert(start>=front().timestamp and stop<=back().timestamp);

	Trajectory ssub;
	for (auto it=begin(); it!=end(); ++it) {
		auto &p = *it;
		if (start<=p.timestamp and p.timestamp<=stop)
			ssub.push_back(p);
	}

	return ssub;
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


PoseTimestamp
PoseTimestamp::operator* (const Pose &t)
{
	Pose P = static_cast<Pose&>(*this) * t;
	return PoseTimestamp(P, this->timestamp);
}

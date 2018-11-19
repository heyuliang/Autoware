/*
 * LidarScanBag.cpp
 *
 *  Created on: Nov 19, 2018
 *      Author: sujiwo
 */


#include "datasets/LidarScanBag.h"


using namespace std;
using velodyne_rawdata::VPoint;
using velodyne_rawdata::VPointCloud;
using pcl::PointCloud;
using pcl::PointXYZ;


/*
 * XXX: These values may need to be adjusted
 */
const float
	velodyneMinRange = 2.0,
	velodyneMaxRange = 130,
	velodyneViewDirection = 0,
	velodyneViewWidth = 2*M_PI;


template<class PointT>
static
PointCloud<PointXYZ>::Ptr
convertToExternal (const PointCloud<PointT> &cloudSrc)
{
	const int w=cloudSrc.width, h=cloudSrc.height;
	PointCloud<PointXYZ>::Ptr cloudExt (new PointCloud<PointXYZ>(w*h, 1));

	if (h==1) for (int i=0; i<w; ++i) {
		cloudExt->at(i).x = cloudSrc.at(i).x;
		cloudExt->at(i).y = cloudSrc.at(i).y;
		cloudExt->at(i).z = cloudSrc.at(i).z;
	}

	else for (int i=0; i<w; ++i) {
		for (int j=0; j<h; ++j) {
			cloudExt->at(i*w + j).x = cloudSrc.at(j, i).x;
			cloudExt->at(i*w + j).y = cloudSrc.at(j, i).y;
			cloudExt->at(i*w + j).z = cloudSrc.at(j, i).z;
		}
	}

	return cloudExt;
}


LidarScanBag::LidarScanBag(
	rosbag::Bag const &bag,
	const std::string &topic,
	const std::string &lidarCalibFile,
	const ros::Time &startTime,
	const ros::Time &endTime) :

		RandomAccessBag(bag, topic, startTime, endTime),
		data_(new velodyne_rawdata::RawData())

{
	if (data_->setupOffline(lidarCalibFile, velodyneMaxRange, velodyneMinRange)
		== -1)
		throw runtime_error("Unable to set velodyne converter");

	data_->setParameters(velodyneMinRange, velodyneMaxRange, velodyneViewDirection, velodyneViewWidth);
}


LidarScanBag
LidarScanBag::subset(const ros::Time &start, ros::Duration &d) const
{

}


pcl::PointCloud<pcl::PointXYZ>::ConstPtr
LidarScanBag::convertMessage(velodyne_msgs::VelodyneScan::ConstPtr bagmsg)
{
	VPointCloud::Ptr outPoints(new VPointCloud);
	outPoints->header.stamp = pcl_conversions::toPCL(bagmsg->header).stamp;
	outPoints->header.frame_id = bagmsg->header.frame_id;
	outPoints->height = 1;

	for (int i=0; i<bagmsg->packets.size(); ++i) {
		data_->unpack(bagmsg->packets[i], *outPoints, bagmsg->packets.size());
	}

	PointCloud<PointXYZ>::Ptr cloudTmp = convertToExternal(*outPoints);
	if (filtered)
	// These are the best value I know of
		return VoxelGridFilter(cloudTmp, 0.2, 3.0);
	else
		return cloudTmp;
}


pcl::PointCloud<pcl::PointXYZ>::ConstPtr
LidarScanBag::at (int position)
{
	auto msgP = RandomAccessBag::at<velodyne_msgs::VelodyneScan>(position);
	return convertMessage(msgP);
}


pcl::PointCloud<pcl::PointXYZ>::ConstPtr
LidarScanBag::VoxelGridFilter (
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr vcloud,
	double voxel_leaf_size,
	double measurement_range)
{
	PointCloud<PointXYZ>::Ptr filteredGridCLoud(new PointCloud<PointXYZ>);

	assert(voxel_leaf_size>=0.1);
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
	voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
	voxel_grid_filter.setInputCloud(vcloud);
	voxel_grid_filter.filter(*filteredGridCLoud);

	return filteredGridCLoud;
}

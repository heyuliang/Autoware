/*
 * LidarScanBag.h
 *
 *  Created on: Aug 15, 2018
 *      Author: sujiwo
 */

#ifndef _LIDARSCANBAG_H
#define _LIDARSCANBAG_H

#include <exception>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <velodyne_pointcloud/rawdata.h>

#include "RandomAccessBag.h"


class LidarScanBag : public RandomAccessBag
{
public:

	typedef std::shared_ptr<LidarScanBag> Ptr;

	LidarScanBag(
		rosbag::Bag const &bag, const std::string &topic,
		const std::string &lidarCalibFile,
		const ros::Time &startTime = ros::TIME_MIN,
		const ros::Time &endTime = ros::TIME_MAX);

	LidarScanBag subset(const ros::Time &start, ros::Duration &d) const;

	pcl::PointCloud<pcl::PointXYZ>::ConstPtr
	at (int position);

	inline
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr
	atDurationSecond (const double S)
	{
		return at (getPositionAtDurationSecond(S));
	}


	bool filtered = false;

protected:
	boost::shared_ptr<velodyne_rawdata::RawData> data_;

	pcl::PointCloud<pcl::PointXYZ>::ConstPtr
	convertMessage(velodyne_msgs::VelodyneScan::ConstPtr bagmsg);

	pcl::PointCloud<pcl::PointXYZ>::ConstPtr
	VoxelGridFilter (
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr vcloud,
		double voxel_leaf_size,
		double measurement_range);

};



#endif /* _LIDARSCANBAG_H */

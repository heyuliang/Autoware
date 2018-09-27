/*
 * MeidaiLocalizerNDT.cpp
 *
 *  Created on: Sep 16, 2018
 *      Author: sujiwo
 */


#include <datasets/MeidaiBagDataset.h>
#include <iostream>
#include <string>
#include <vector>
#include <exception>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <velodyne_pointcloud/rawdata.h>

#include "utilities.h"
#include "NdtLocalizer.h"


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

const NdtLocalizerInitialConfig NuInitialConfig = {
	0,0,0, 0,0,0
};


class VelodynePreprocessor
{
public:

	VelodynePreprocessor(const string &lidarCalibFile):

		 data_(new velodyne_rawdata::RawData())
	{
		if (data_->setupOffline(lidarCalibFile, velodyneMaxRange, velodyneMinRange)
			== -1)
			throw runtime_error("Unable to set velodyne converter");

		data_->setParameters(velodyneMinRange, velodyneMaxRange, velodyneViewDirection, velodyneViewWidth);
	}

	PointCloud<PointXYZ>::ConstPtr
	convertMessage(velodyne_msgs::VelodyneScan::ConstPtr bagmsg)
	{
		VPointCloud::Ptr outPoints(new VPointCloud);
		outPoints->header.stamp = pcl_conversions::toPCL(bagmsg->header).stamp;
		outPoints->header.frame_id = bagmsg->header.frame_id;
		outPoints->height = 1;

		for (int i=0; i<bagmsg->packets.size(); ++i) {
			data_->unpack(bagmsg->packets[i], *outPoints, bagmsg->packets.size());
		}

		PointCloud<PointXYZ>::Ptr cloudTmp = convertToExternal(*outPoints);
		// These are the best value I know of
		return VoxelGridFilter(cloudTmp, 0.2, 3.0);
	}

	PointCloud<PointXYZ>::ConstPtr
	VoxelGridFilter (PointCloud<PointXYZ>::ConstPtr vcloud, double voxel_leaf_size, double measurement_range)
	{
		PointCloud<PointXYZ>::Ptr filteredGridCLoud(new PointCloud<PointXYZ>);

		assert(voxel_leaf_size>=0.1);
		pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
		voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
		voxel_grid_filter.setInputCloud(vcloud);
		voxel_grid_filter.filter(*filteredGridCLoud);

		return filteredGridCLoud;
	}


protected:
	boost::shared_ptr<velodyne_rawdata::RawData> data_;

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
};


void
createTrajectoryFromNDT (RandomAccessBag &bagsrc, Trajectory &resultTrack, const Trajectory &gnssTrack, const string &velodyneParamFile, const string &pcdMapFile)
{
	if (bagsrc.getTopic() != "/velodyne_packets")
		throw runtime_error("Not Velodyne bag");

	VelodynePreprocessor VP(velodyneParamFile);
	NdtLocalizer lidarLocalizer(NuInitialConfig);
	lidarLocalizer.loadMap(pcdMapFile);

	bool initialized=false;
	auto time0 = bagsrc.at<velodyne_msgs::VelodyneScan>(0)->header.stamp;

	// XXX: How to catch NDT's failure ?
	for (uint32_t ip=0; ip<bagsrc.size(); ++ip) {

		Pose cNdtPose;
		auto cMsg = bagsrc.at<velodyne_msgs::VelodyneScan>(ip);
		auto cscan = VP.convertMessage(cMsg);

		if (!initialized) {
			try {

				auto cGnssPos = gnssTrack.at(cMsg->header.stamp);
				Vector3d p = cGnssPos.position();
				Quaterniond q = cGnssPos.orientation();
				lidarLocalizer.putEstimation(cGnssPos);
				cNdtPose = lidarLocalizer.localize(cscan);
				initialized = true;

			} catch (out_of_range &e) {
				continue;
			} catch (exception &e) {
				cerr << "Unknown error\n";
				continue;
			}
		}

		else {
			cNdtPose = lidarLocalizer.localize(cscan);
		}

		PoseTimestamp tpose (cNdtPose);
		tpose.timestamp = cMsg->header.stamp;
//		cerr << fixed;
//		auto td = tpose.timestamp - time0;
//		cerr << tpose.timestamp.toSec() << ' '
//			 <<	tpose.position().x() << " " <<
//				tpose.position().y() << " " <<
//				tpose.position().z() << endl;
		resultTrack.push_back(tpose);
	}

	return;
}

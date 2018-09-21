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
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/rawdata.h>
#include <ndt.h>


using namespace std;
using velodyne_rawdata::VPointCloud;


const float
	velodyneMinRange = 2.0,
	velodyneMaxRange = 130;


class VelodynePreprocessor
{
public:

	VelodynePreprocessor(const string &lidarCalibFile):

		 data_(new velodyne_rawdata::RawData())
	{
		data_->setupOffline(lidarCalibFile, velodyneMaxRange, velodyneMinRange);
	}

	VPointCloud::ConstPtr
	convert(velodyne_msgs::VelodyneScan::ConstPtr bagmsg)
	{
	    VPointCloud::Ptr outPoints(new VPointCloud);
	    outPoints->header.stamp = pcl_conversions::toPCL(bagmsg->header).stamp;
	    outPoints->header.frame_id = bagmsg->header.frame_id;
	    outPoints->height = 1;

	    for (int i=0; i<bagmsg->packets.size(); ++i) {
	    	data_->unpack(bagmsg->packets[i], *outPoints, bagmsg->packets.size());
	    }

	    return outPoints;
	}

protected:
	boost::shared_ptr<velodyne_rawdata::RawData> data_;
};


void
createTrajectoryFromNDT (RandomAccessBag &bagsrc, Trajectory &resultTrack, const Trajectory &gnssTrack)
{
	if (bagsrc.getTopic() != "/velodyne_packets")
		throw runtime_error("Not Velodyne bag");

	const string paramFileTest = "/home/sujiwo/Autoware/ros/src/computing/perception/localization/packages/vmml/params/64e-S2.yaml";
	VelodynePreprocessor VP(paramFileTest);

	bool initialized=false;
	for (uint32_t ip=0; ip<bagsrc.size(); ++ip) {

		auto cMsg = bagsrc.at<velodyne_msgs::VelodyneScan>(ip);
		auto clx = VP.convert(cMsg);

		if (clx->size()>0) {
			pcl::io::savePCDFileBinary("/tmp/p.pcd", *clx);
		}

		auto cGnssPos = gnssTrack.at(cMsg->header.stamp);

	}

	return;
}

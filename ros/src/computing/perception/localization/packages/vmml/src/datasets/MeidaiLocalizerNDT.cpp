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


const NdtLocalizerInitialConfig NuInitialConfig = {
	0,0,0, 0,0,0
};




void
createTrajectoryFromNDT (LidarScanBag &bagsrc, Trajectory &resultTrack, const Trajectory &gnssTrack, const string &velodyneParamFile, const string &pcdMapFile)
{
	if (bagsrc.getTopic() != "/velodyne_packets")
		throw runtime_error("Not Velodyne bag");

	bagsrc.filtered = true;
	NdtLocalizer lidarLocalizer(NuInitialConfig);
	lidarLocalizer.loadMap(pcdMapFile);
	resultTrack.clear();

	bool initialized=false;
	auto time0 = bagsrc.timeAt(0);

	// XXX: How to catch NDT's failure ?
	uint32_t N = bagsrc.size();
	for (uint32_t ip=0; ip<N; ++ip) {

		Pose cNdtPose;
		auto cscan = bagsrc.at(ip);
		auto scanTime = bagsrc.timeAt(ip);
		cout << ip+1 << " / " << N << "   \r" << flush;

		try {

			if (!initialized) {
				auto cGnssPos = gnssTrack.at(scanTime);
				if (lidarLocalizer.isPointInsideMap(cGnssPos.position())==false)
					throw out_of_range("Initialization point lies outside map");

				Vector3d p = cGnssPos.position();
				Quaterniond q = cGnssPos.orientation();
				lidarLocalizer.putEstimation(cGnssPos);
				cNdtPose = lidarLocalizer.localize(cscan);
				initialized = true;

			}

			else {
				cNdtPose = lidarLocalizer.localize(cscan);
				if (lidarLocalizer.isPointInsideMap(cNdtPose.position())==false)
					throw out_of_range("Continuation point lies outside map; resetting");
			}

			PoseTimestamp tpose (cNdtPose);
			tpose.timestamp = scanTime;
			resultTrack.push_back(tpose);

		} catch (out_of_range &e) {
			cerr << "Error: " << e.what() << endl;
			resultTrack.clear();
			initialized = false;
		}
	}

	return;
}

/*
 * test_bag.cpp
 *
 *  Created on: Aug 10, 2018
 *      Author: sujiwo
 */


#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <datasets/MeidaiBagDataset.h>

#include "utilities.h"


using namespace std;
using Eigen::Vector3d;


#define Precision 10


// Velodyne HDL-64 calibration file and map
const string paramFileTest = "/home/sujiwo/Autoware/ros/src/computing/perception/localization/packages/vmml/params/64e-S2.yaml";
const string meidaiMapPcd = "/home/sujiwo/Data/NagoyaUniversityMap/bin_meidai_ndmap.pcd";


int main (int argc, char *argv[])
{
	MeidaiBagDataset dataset("/home/sujiwo/Data/log_2016-12-26-13-21-10-filtered.bag");
	const Trajectory &gnssTraj = dataset.getGnssTrajectory();
	auto velBag = dataset.getVelodyneBag();

	Trajectory ndtTrack;
	createTrajectoryFromNDT(*velBag, ndtTrack, gnssTraj, paramFileTest, meidaiMapPcd);

//	dataset.forceCreateCache();

	return 0;
}

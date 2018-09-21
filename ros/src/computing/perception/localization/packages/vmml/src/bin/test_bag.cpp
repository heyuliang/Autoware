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

string dumpVector(const Vector3d &v)
{
	stringstream s;
	s.precision(Precision);
	s << v.x() << " " << v.y() << " " << v.z();
	return s.str();
}


string dumpVector(const Quaterniond &v)
{
	stringstream s;
	s.precision(Precision);
	s << v.x() << " " << v.y() << " " << v.z() << ' ' << v.w();
	return s.str();
}


int main (int argc, char *argv[])
{
	MeidaiBagDataset dataset("/home/sujiwo/Data/log_2016-12-26-13-21-10-filtered.bag");
	const Trajectory &gnssTraj = dataset.getGnssTrajectory();
	auto velBag = dataset.getVelodyneBag();

	Trajectory ndtTrack;
	createTrajectoryFromNDT(*velBag, ndtTrack, gnssTraj);

//	dataset.forceCreateCache();

	return 0;
}

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

#include <datasets/RandomAccessBag.h>
#include "utilities.h"


using namespace std;
using Eigen::Vector3d;


#define Precision 10


// Velodyne HDL-64 calibration file and map
const string bagTestPath = "/home/sujiwo/Data/log_2016-12-26-13-21-10-filtered.bag";


int main (int argc, char *argv[])
{
	rosbag::Bag bagfd (bagTestPath);
	RandomAccessBag imageBag (bagfd, "/camera1/image_raw");
	auto sz = imageBag.size();

	RandomAccessBag imageBag2 (bagfd, "/camera1/image_raw", imageBag.startTime(), imageBag.startTime()+ros::Duration(5));
	sz = imageBag2.size();

	return 0;
}

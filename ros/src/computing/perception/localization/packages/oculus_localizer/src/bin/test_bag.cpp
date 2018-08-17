/*
 * test_bag.cpp
 *
 *  Created on: Aug 10, 2018
 *      Author: sujiwo
 */


#include <cstdio>
#include <iostream>
#include <vector>
#include <string>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <rosbag/structures.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>

#include "datasets/RandomAccessBag.h"
#include "datasets/MeidaiBag.h"
#include "utilities.h"


using namespace std;
using Eigen::Vector3d;




int main (int argc, char *argv[])
{
	rosbag::Bag bagstore;
	bagstore.open("/home/sujiwo/Data/log_2016-12-26-13-21-10-filtered.bag");

	RandomAccessBag ramImageBag (bagstore, "/camera1/image_raw");
	auto img0 = ramImageBag.at<sensor_msgs::Image>(0);

	RandomAccessBag ramGnssBag (bagstore, "/nmea_sentence");
	Trajectory track;
	createTrajectoryFromGnssBag(ramGnssBag, track);

	for (auto pts: track) {
		printf ("%f %f %f\n", pts.position().x(), pts.position().y(), pts.position().z());
	}

	return 0;
}

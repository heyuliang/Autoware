/*
 * test_bag.cpp
 *
 *  Created on: Aug 10, 2018
 *      Author: sujiwo
 */


#include <iostream>
#include <vector>
#include <string>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <datasets/MeidaiBagDataset.h>

#include "utilities.h"


using namespace std;
using Eigen::Vector3d;


int main (int argc, char *argv[])
{
	MeidaiBagDataset dataset("/home/sujiwo/Data/log_2016-12-26-13-21-10-filtered.bag");

	return 0;
}

/*
 * test_tf.cpp
 *
 *  Created on: Jul 31, 2018
 *      Author: sujiwo
 */

#include <iostream>
#include <sstream>
#include <cstdio>
#include <string>
#include <cmath>

#include "utilities.h"


using namespace std;
using namespace Eigen;


const double PI2 = M_PI / 2;


int main (int argc, char *argv[])
{
	Pose world_robot(5,5,0, 0,0,0);
	TTransform robot_camera(0,0.3,0.3, -PI2,0,0);

	const Pose world_camera = world_robot * robot_camera;

	cout << dumpVector(world_camera);
	return 0;
}

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

#include "utilities.h"


using namespace std;
using namespace Eigen;


int main (int argc, char *argv[])
{
	Pose A(1,0,0, 0,0,0);
	TTransform V(0,0,0, 0.79,0,0);

	A = V*A;

//	cout << A << endl;
	return 0;
}

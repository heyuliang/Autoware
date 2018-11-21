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
#include <algorithm>

#include "MTContainers.h"


using namespace std;

#define PI_2 M_PI/2

int main (int argc, char *argv[])
{
	Mt::vector<float> myfloats;
	myfloats.push_back(5.0);
	myfloats.push_back(9.0);
	myfloats.push_back(15.0);
	myfloats.push_back(25.0);

	cout << myfloats[2] << endl;

	Mt::map<int, double> keyvalues;
	keyvalues.insert(make_pair(0, M_PI));
	keyvalues.insert(make_pair(5, M_PI*5));
	keyvalues.insert(make_pair(8, -1e2));

	cout << keyvalues.at(5) << endl;
}

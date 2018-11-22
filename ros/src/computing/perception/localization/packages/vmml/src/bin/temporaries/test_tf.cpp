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


vector<double> createRandom (int N)
{
	vector<double> rndlist;

	for (int i=0; i<N; ++i)
		rndlist.push_back(double(i) * 2);

	return rndlist;
}


Mt::map<int, double> createRandomMap (int N)
{
	Mt::map<int,double> nMap;

	for (int i=0; i<N; ++i) {
		nMap[i] = double(N) * PI_2;
	}

	return nMap;
}


int main (int argc, char *argv[])
{
	Mt::vector<double> myfloats = createRandom(7);

	cout << myfloats[2] << endl;

	Mt::map<int, double> keyvalues = createRandomMap(7);
	keyvalues[0] = -1;

	cout << keyvalues.at(5) << endl;
}

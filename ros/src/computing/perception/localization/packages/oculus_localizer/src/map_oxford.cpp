/*
 * map_oxford.cpp
 *
 *  Created on: Jul 30, 2018
 *      Author: sujiwo
 */


#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "OxfordDataset.h"
#include "MapBuilder2.h"

using namespace std;
using namespace Eigen;


MapBuilder2 *builder;
const double
	translationThrs = 1.0,	// meter
	rotationThrs = 0.04;	// == 2.5 degrees


InputFrame createInputFrame(OxfordDataItem &d)
{
	cv::Mat i=d.getImage();
	return InputFrame(i, d.groundTruth.position(), d.groundTruth.orientation());
}


void buildMap (OxfordDataset &dataset)
{
	builder = new MapBuilder2;

	// Find two initializer frame
	OxfordDataItem d0 = dataset.at(0);
	OxfordDataItem d1;
	int i=1;
	while (true) {
		d1 = dataset.at(1);
		double runTrans, runRot;
		d1.groundTruth.displacement(d0.groundTruth, runTrans, runRot);
		if (runTrans>=translationThrs or runRot>=rotationThrs)
			break;
	}

	InputFrame frame0 = createInputFrame(d0);
	InputFrame frame1 = createInputFrame(d1);
	builder->initialize(frame0, frame1);

	delete builder;
}


int main (int argc, char *argv[])
{
	OxfordDataset oxf(argv[1], "/home/sujiwo/Sources/robotcar-dataset-sdk/models");
	oxf.dump();
	return 0;
}

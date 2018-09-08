/*
 * map_oxford.cpp
 *
 *  Created on: Jul 30, 2018
 *      Author: sujiwo
 */


#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "datasets/OxfordDataset.h"
#include "utilities.h"
#include "Viewer.h"
#include "MapBuilder2.h"

using namespace std;
using namespace Eigen;


MapBuilder2 *builder = NULL;
Viewer *imgViewer;


const string
	mapFileOutput = "/tmp/oxford1.map";


InputFrame createInputFrame(OxfordDataItem &d)
{
	cv::Mat i=d.getImage();
	// Oxford datasets output is RGB
	cv::cvtColor(i, i, CV_BGR2GRAY, 1);
	InputFrame f(
		i,
		d.groundTruth.position(),
		d.groundTruth.orientation(),
		// Force Keyframe ID using timestamp. This way, we can refer to original
		// image for display purpose
		static_cast<kfid>(d.timestamp));
	f.tm = d.getTimestamp();

	return f;
}


void buildMap2
(OxfordDataset &dataset)
{
	builder = new MapBuilder2;
	builder->addCameraParam(dataset.getCameraParameter());

	imgViewer = new Viewer (dataset);
	imgViewer->setMap(builder->getMap());
	dataItemId currentItemId;

	MapBuilder2::frameCallback frmCallback =
	[&] (const InputFrame &f)
	{
		imgViewer->update(currentItemId, builder->getCurrentKeyFrameId());
		cout << currentItemId << " / " << dataset.size() << endl;
	};
	builder->registerFrameCallback(frmCallback);

	for (int framePtr=0; framePtr<dataset.size(); framePtr++) {
		OxfordDataItem &dx = dataset.at(framePtr);
		currentItemId = dx.getId();
		InputFrame frame = createInputFrame(dx);
		builder->track2(frame);
	}
}


void buildMap (OxfordDataset &dataset)
{
	if (builder==NULL) {
		builder = new MapBuilder2;
		builder->addCameraParam(dataset.getCameraParameter());
	}

	imgViewer = new Viewer(dataset);
	imgViewer->setMap(builder->getMap());

	// Find two initializer frames
	OxfordDataItem d0, d1;
	InputFrame frame0, frame1;
	int iptr;
	for (iptr=0; iptr<dataset.size(); iptr++) {
		d0 = dataset.at(iptr);
		frame0 = createInputFrame(d0);
		auto normcdf = cdf(frame0.image);
		if (normcdf[127]<0.25)
			continue;
		else {
			break;
		}
	}
	for (iptr+=1; iptr<dataset.size(); iptr++) {
		d1 = dataset.at(iptr);
		frame1 = createInputFrame(d1);
		auto normcdf = cdf(frame1.image);
		if (normcdf[127]<0.25)
			continue;
		double runTrans, runRot;
		frame1.getPose().displacement(frame0.getPose(), runTrans, runRot);
		if (runTrans>=translationThrs or runRot>=rotationThrs)
			break;
	}

	builder->initialize(frame0, frame1);
	imgViewer->update(d1.getId(), builder->getCurrentKeyFrameId());

	OxfordDataItem *anchor = &d1;

	for (int i=2; i<dataset.size(); i++) {

		OxfordDataItem &dCurrent = dataset.at(i);
		double diffTrans, diffRotn;
		anchor->groundTruth.displacement(dCurrent.groundTruth, diffTrans, diffRotn);
		if (diffTrans<translationThrs and diffRotn<rotationThrs)
			continue;

		// XXX: Store Anchor frame in MapBuilder2 itself
		InputFrame frameCurrent = createInputFrame(dCurrent);
		builder->track(frameCurrent);
		anchor = &dCurrent;
		cerr << anchor->getId() << endl;
		imgViewer->update(dCurrent.getId(), builder->getCurrentKeyFrameId());
	}

	builder->build();
}


int main (int argc, char *argv[])
{
	OxfordDataset oxf(argv[1], "/home/sujiwo/Sources/robotcar-dataset-sdk/models");
	OxfordDataItem m0 = oxf.at(0);
	OxfordDataset oxfSubset;
	oxf.setZoomRatio(0.5);

	double start, howlong;
	if (argc==3) {
		start=stod(argv[2]),
		howlong=stod(argv[3]);
		oxfSubset = oxf.timeSubset(start, howlong);
	}
	else {
		oxfSubset = oxf;
	}

	string stname = oxfSubset.getName();
	buildMap2 (oxfSubset);
	builder->getMap()->save(mapFileOutput);
	cout << "Saved to " << mapFileOutput << endl;
//	oxf.dump();
	return 0;
}

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
#include "Viewer.h"
#include "MapBuilder2.h"

using namespace std;
using namespace Eigen;


MapBuilder2 *builder;
Viewer *imgViewer;

const double
	translationThrs = 1.0,	// meter
	rotationThrs = 0.04;	// == 2.5 degrees


class OxfordDataSubset : public OxfordDataset
{
public:
	OxfordDataSubset(const std::string &dirpath, const std::string &modelDir, double startTimeOffsetSecond=0, double mappingDurationSecond=-1):
		OxfordDataset(dirpath, modelDir)
	{
		double absStartTimeSecond = double(stereoTimestamps[0]/1e6) + startTimeOffsetSecond;
		if (mappingDurationSecond<0) {
			double lastTime = double(stereoTimestamps.back()) / 1e6;
			mappingDurationSecond = lastTime - absStartTimeSecond;
		}

		auto it = stereoTimestamps.begin();
		for (; it!=stereoTimestamps.end(); ++it) {
			double ts = double(*it) / 1e6;
			if (ts>=absStartTimeSecond) {
				if (ts > absStartTimeSecond + mappingDurationSecond)
					break;
				_timerange.push_back(*it);
			}
		}
	}

	OxfordDataItem &at(const int i) const
	{
		timestamp_t ts = _timerange.at(i);
		return const_cast<OxfordDataItem&>(stereoRecords.at(ts));
	}

	size_t size() const
	{ return _timerange.size();	}

protected:
	vector<timestamp_t> _timerange;
};



InputFrame createInputFrame(OxfordDataItem &d)
{
	cv::Mat i=d.getImage();
	// Oxford datasets output is RGB
	cv::cvtColor(i, i, CV_RGB2GRAY, 1);
	return InputFrame(i, d.groundTruth.position(), d.groundTruth.orientation());
}


void buildMap (OxfordDataSubset &dataset)
{
	builder = new MapBuilder2;
	builder->addCameraParam(dataset.getCameraParameter());

	imgViewer = new Viewer(dataset);
	imgViewer->setMap(builder->getMap());

	// Find two initializer frames
	OxfordDataItem d0 = dataset.at(0);
	OxfordDataItem d1;
	int i=1;
	while (true) {
		d1 = dataset.at(i);
		double runTrans, runRot;
		d1.groundTruth.displacement(d0.groundTruth, runTrans, runRot);
		if (runTrans>=translationThrs or runRot>=rotationThrs)
			break;
		i++;
	}

	InputFrame frame0 = createInputFrame(d0);
	InputFrame frame1 = createInputFrame(d1);
	builder->initialize(frame0, frame1);

	delete builder;
}


int main (int argc, char *argv[])
{
	OxfordDataSubset oxf(argv[1], "/home/sujiwo/Sources/robotcar-dataset-sdk/models", 0, 30.0);
	buildMap (oxf);
//	oxf.dump();
	return 0;
}

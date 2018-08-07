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


class OxfordDataSubset : OxfordDataset
{
public:
	OxfordDataSubset(const std::string &dirpath, const std::string &modelDir, float startTimeSec, float mappingDurationSec=-1):
		OxfordDataset(dirpath, modelDir)
	{
		if (startTimeSec < float(stereoTimestamps)/1e6)
			throw runtime_error("Invalid start timestamp");
		if (startTimeSec+mappingDurationSec > float(stereoTimestamps[stereoTimestamps.size()-1])/1e6)
			throw runtime_error("Invalid duration");

		auto it = stereoTimestamps.begin();
		for (; it!=stereoTimestamps.end(); ++it) {
			float ts = *it / 1e6;
			if (ts>startTimeSec) {
				if (startTimeSec + mappingDurationSec < ts)
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
	return InputFrame(i, d.groundTruth.position(), d.groundTruth.orientation());
}


void buildMap (OxfordDataSubset &dataset)
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
	OxfordDataSubset oxf(argv[1], "/home/sujiwo/Sources/robotcar-dataset-sdk/models", 0, 30.0);
	oxf.dump();
	return 0;
}

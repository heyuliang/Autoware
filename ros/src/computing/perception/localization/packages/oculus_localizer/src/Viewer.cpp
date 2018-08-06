/*
 * Viewer.cpp
 *
 *  Created on: Jul 10, 2018
 *      Author: sujiwo
 */

#include <map>
#include "Viewer.h"
#include "VMap.h"
#include "utilities.h"
#include "KeyFrame.h"


using namespace std;


static string wndName ("Viewer");
static cv::Vec3b kpColor(255,0,0);


Viewer::Viewer (const GenericDataset &genset) :
	dataset(genset),
	cMap(NULL)
{
	cv::namedWindow(wndName);
}


Viewer::~Viewer()
{
	cv::destroyWindow(wndName);
}


void
Viewer::setMap (VMap *m)
{ cMap = m; }


vector<cv::KeyPoint>
collectAllKeypoints (const KeyFrame *kf, const vector<kpid> &allKpIds)
{
	vector<cv::KeyPoint> allKp;
	for (auto &i: allKpIds) {
		allKp.push_back(kf->getKeyPointAt(i));
	}
	return allKp;
}


void
Viewer::update (int dataItemId, kfid frameId)
{
	assert (cMap != NULL);
	KeyFrame *kf = cMap->keyframe(frameId);
	cv::Mat imagebuf = dataset.at(dataItemId).getImage();

	// Draw visible map points
	const map<mpid,kpid> visibleMp = cMap->allMapPointsAtKeyFrame(frameId);
	if (visibleMp.size() != 0) {
		vector<kpid> allMpKpIds = allValues(visibleMp);
		auto allKeypoints = collectAllKeypoints(kf, allMpKpIds);
		cv::drawKeypoints(imagebuf, allKeypoints, imagebuf, kpColor);
	}

	cv::imshow(wndName, imagebuf);
	cv::waitKey(1);
}

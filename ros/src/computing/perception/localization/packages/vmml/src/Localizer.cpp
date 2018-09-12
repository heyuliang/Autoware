/*
 * Localizer.cpp
 *
 *  Created on: Jul 23, 2018
 *      Author: sujiwo
 */

#include "Localizer.h"


Localizer::Localizer(VMap *parentMap, bool emptyMask) :
	sourceMap(parentMap),
	featureDetector(parentMap->getFeatureDetector()),
	imgDb(parentMap->getImageDB())
{
	if (emptyMask==false)
		setMask(sourceMap->getMask());
	else
		setMask(cv::Mat());
}


Localizer::~Localizer()
{}


void
Localizer::setCameraParameterFromId (int cameraId)
{
	localizerCamera = sourceMap->getCameraParameter(cameraId);
}


kfid
Localizer::detect (cv::Mat &frmImg)
{
	cv::Mat rzImg;
	if (frmImg.cols != localizerCamera.width) {
		float ratio = float(localizerCamera.width) / float(frmImg.cols);
		cv::resize(frmImg, rzImg, cv::Size(), ratio, ratio, cv::INTER_CUBIC);
	}
	else
		rzImg = frmImg;

	Frame frm (rzImg, this);
	vector<kfid> placeCandidates = imgDb->findCandidates(frm);

	// XXX: Check each candidate using projection

	return placeCandidates[0];
}


float
Localizer::projectionCheck (const Frame &frame, const kfid &keyframe)
const
{

}

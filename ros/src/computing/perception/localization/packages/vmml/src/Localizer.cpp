/*
 * Localizer.cpp
 *
 *  Created on: Jul 23, 2018
 *      Author: sujiwo
 */

#include "Localizer.h"
#include "utilities.h"


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


int
Localizer::SearchBoW (const kfid &k, Frame &frame, vector<mpid> &vpMapPts)
{
	auto mapPtsInKF = sourceMap->allMapPointsAtKeyFrame(k);
	auto KeyFrameFeatureVec = imgDb->getFeatureVectorFromKeyFrame(k);
	auto keyPtMapPoint = reverseMap(mapPtsInKF);
	int matches = 0;

	auto KFit = KeyFrameFeatureVec.begin();
	auto Fit = frame.getFeatureVector().begin();

	while (KFit!=KeyFrameFeatureVec.end() and Fit!=frame.getFeatureVector().end()) {

		if (KFit->first == Fit->first) {

			const auto vIndicesKF = KFit->second;
			const auto vIndicesF = Fit->second;

			for (uint32_t iKF=0; iKF<vIndicesKF.size(); iKF++) {
				auto realKpIdxKF = vIndicesKF[iKF];
				try {
					mpid nMpId = keyPtMapPoint.at(realKpIdxKF);
				} catch (out_of_range &e) {
					continue;
				}

				const cv::Mat &descKf = sourceMap->keyframe(k)->getDescriptorAt(realKpIdxKF);

				int bestDist1 = 256;
				int bestIdxF = -1;
				int bestDist2 = 256;

				for (uint32_t iF=0; iF<vIndicesF.size(); iF++) {
					const auto realKpIdxF = vIndicesF[iF];
				}
			}

			KFit ++;
			Fit ++;
		}

		else if (KFit->first < Fit->first) {
			KFit = KeyFrameFeatureVec.lower_bound(Fit->first);
		}

		else {
			Fit = frame.getFeatureVector().lower_bound(KFit->first);
		}
	}


	return matches;
}

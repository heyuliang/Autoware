/*
 * Localizer.cpp
 *
 *  Created on: Jul 23, 2018
 *      Author: sujiwo
 */

#include "KeyFrame.h"
#include "Localizer.h"
#include "MapPoint.h"
#include "utilities.h"


const int OrbDescriptorDistanceUpperThreshold = 100;
const int OrbDescriptorDistanceLowerThreshold = 50;
const int OrbDescriptorHistogramLength = 30;


using Eigen::Vector2d;
using Eigen::Vector3d;



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

	Frame frame (rzImg, this);
	vector<kfid> placeCandidates = imgDb->findCandidates(frame);

	// for debugging
	vector<dataItemId> srcInfo(placeCandidates.size());
	for (int i=0; i<placeCandidates.size(); ++i) {
		srcInfo[i] = sourceMap->keyframe(placeCandidates[i])->getSourceItemId();
	}

/*
 *  We need this section to match keypoint from Candidate KeyFrame and (corresponding map point) to keypoint in Frame
 *  XXX: need alternative, ie. using brute force matching from OpenCV, and check using projection
 */

//	auto bfMatch = cv::BFMatcher::create();

	size_t bestKfScore=0;
	kfid bestKfId;

	for (int i=0; i<placeCandidates.size(); i++) {

		const KeyFrame &kf = *sourceMap->keyframe(placeCandidates[i]);
		vector<FeaturePair> kfMatches;
		KeyFrame::match(kf, frame, kfMatches, sourceMap->getDescriptorMatcher());

		size_t curScore = kfMatches.size();
		if (curScore < 15) {
			continue;
		}

		else if (curScore > bestKfScore) {
			bestKfScore = curScore;
			bestKfId = kf.getId();
		}
	}

	return bestKfId;
}


/*

#define averageProjectionDeviation 2.0

float
Localizer::projectionCheck (const Frame &frame, const kfid &keyframeId)
const
{
	vector<FeaturePair> featurePairs;
	KeyFrame *keyframe = sourceMap->keyframe(keyframeId);

//	KeyFrame::match(*keyframe, frame, featurePairs);
	vector<float> projectionErrors(featurePairs.size());

	for (int i=0; i<featurePairs.size(); i++) {
		auto &pair = featurePairs[i];

		const MapPoint &pt3d = *sourceMap->mappoint(sourceMap->getMapPointByKeypoint(keyframeId, pair.kpid1));
		Vector2d ptProj = keyframe->project(pt3d);
		const float dist2D = (ptProj - Vector2d(pair.keypoint2.x, pair.keypoint2.y)).norm();
		if (dist2D < averageProjectionDeviation) {

		}
	}
}
*/


int
Localizer::SearchBoW (const kfid &k, Frame &frame, set<mpid> &vpMapPts, const float matchNNRatio)
{
	auto mapPtsInKF = sourceMap->allMapPointsAtKeyFrame(k);
	auto KeyFrameFeatureVec = imgDb->getFeatureVectorFromKeyFrame(k);
	auto keyPtMapPoint = reverseMap(mapPtsInKF);
	int matches = 0;
	vpMapPts.clear();

	auto KFit = KeyFrameFeatureVec.begin();
	auto Fit = frame.getFeatureVector().begin();

	while (KFit!=KeyFrameFeatureVec.end() and Fit!=frame.getFeatureVector().end()) {

		if (KFit->first == Fit->first) {

			const auto vIndicesKF = KFit->second;
			const auto vIndicesF = Fit->second;

			for (uint32_t iKF=0; iKF<vIndicesKF.size(); iKF++) {

				auto realKpIdxKF = vIndicesKF[iKF];
				mpid nMpId;
				try {
					nMpId = keyPtMapPoint.at(realKpIdxKF);
				} catch (out_of_range &e) {
					continue;
				}

				const cv::Mat &descKf = sourceMap->keyframe(k)->getDescriptorAt(realKpIdxKF);

				int bestDist1 = 256;
				int bestIdxF = -1;
				int bestDist2 = 256;

				for (uint32_t iF=0; iF<vIndicesF.size(); iF++) {
					const auto realKpIdxF = vIndicesF[iF];

					if (vpMapPts.find(nMpId) != vpMapPts.end())
						continue;

					const cv::Mat descF = frame.descriptor(realKpIdxF);
					int descDist = ORBDescriptorDistance(descKf, descF);

					if (descDist < bestDist1) {
						bestDist2 = bestDist1;
						bestDist2 = descDist;
						bestIdxF = realKpIdxF;
					} else if (descDist<bestDist2) {
						bestDist2 = descDist;
					}
				}

				if (bestDist1 <= OrbDescriptorDistanceLowerThreshold) {
					if (float(bestDist1) <  matchNNRatio*float(bestDist2)) {
						vpMapPts.insert(nMpId);
						matches++;
						const cv::KeyPoint &kp = sourceMap->keyframe(k)->getKeyPointAt(realKpIdxKF);

						// Check orientation
						// XXX: We skip orientation check
//						float rot = kp.angle - frame.keypoints(bestIdxF);
//						if (rot<0.0)
//							rot += 360.f;
					}
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

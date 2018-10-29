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


void
Localizer::debug_KF_F_Matching (const KeyFrame &keyframe, const Frame &frame, const vector<pair<mpid,kpid>> &mapPtMatchPairs)
{
	const cv::Scalar
		colorGreen(0, 255, 0),
		colorRed(0, 0, 255);

	cv::Mat img = frame.getImage().clone();
	cv::cvtColor(img, img, CV_GRAY2BGR);
	string dpath = "/tmp/frame-" + to_string(keyframe.getSourceItemId()) + ".png";

	for (auto &mpair: mapPtMatchPairs) {
		const kpid _kp = sourceMap->getKeyPointId(keyframe.getId(), mpair.first);
		const cv::KeyPoint k_in_kf = keyframe.getKeyPointAt(_kp);
		const cv::KeyPoint k_in_f = frame.keypoint(mpair.second);
		cv::line(img, k_in_kf.pt, k_in_f.pt, colorGreen, 1);
		cv::circle(img, k_in_kf.pt, 2, colorRed);
	}

	cv::imwrite(dpath, img);
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

	size_t bestKfScore=0;
	kfid bestKfId;
	vector<bool> isValidKfs (placeCandidates.size(), false);

	for (int i=0; i<placeCandidates.size(); i++) {

		const KeyFrame &kf = *sourceMap->keyframe(placeCandidates[i]);
		/*
		 * Some notes on this vector:
		 * This is a pair of mappoint ID to its reflection (as keypoint) in
		 * currently investigated frame
		 */
		vector<pair<mpid,kpid>> mapPointMatches;

		int numMatches = SearchBoW (kf, frame, mapPointMatches);
		if (numMatches >= 15) {
			isValidKfs[i] = true;
			debug_KF_F_Matching(kf, frame, mapPointMatches);
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


/*
 * Match the map points in kf to keypoints in frame
 */
int
Localizer::SearchBoW (const KeyFrame &kf, Frame &frame, vector<pair<mpid,kpid>> &mapPtMatchPairs, const float matchNNRatio)
{
	auto mapPtsInKF = sourceMap->allMapPointsAtKeyFrame(kf.getId());
	auto KeyFrameFeatureVec = imgDb->getFeatureVectorFromKeyFrame(kf.getId());
	auto keyPtMapPoint = reverseMap(mapPtsInKF);
	int matches = 0;

	set<mpid> vpMapPts;
	set<kpid> vFrameKp;
	mapPtMatchPairs.clear();

	auto KFit = KeyFrameFeatureVec.begin();
	auto KFend = KeyFrameFeatureVec.end();
	auto Fit = frame.getFeatureVector().begin();
	auto Fend = frame.getFeatureVector().end();

	while (KFit!=KFend and Fit!=Fend) {

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

				const cv::Mat &descKf = kf.getDescriptorAt(realKpIdxKF);

				int bestDist1 = 256;
				int bestIdxF = -1;
				int bestDist2 = 256;

				kpid realKpIdxF;
				for (uint32_t iF=0; iF<vIndicesF.size(); ++iF) {
					realKpIdxF = vIndicesF[iF];

					if (vpMapPts.find(nMpId) != vpMapPts.end())
						continue;

					const cv::Mat descF = frame.descriptor(realKpIdxF);
					int descDist = ORBDescriptorDistance(descKf, descF);

					if (descDist < bestDist1) {
						bestDist2 = bestDist1;
						bestDist1 = descDist;
						bestIdxF = realKpIdxF;
					} else if (descDist<bestDist2) {
						bestDist2 = descDist;
					}
				}

				if (bestDist1 <= OrbDescriptorDistanceLowerThreshold) {
					if (float(bestDist1) <  matchNNRatio*float(bestDist2)) {
						vpMapPts.insert(nMpId);
						matches++;
						const cv::KeyPoint &kp = kf.getKeyPointAt(realKpIdxKF);
						mapPtMatchPairs.push_back (make_pair(nMpId, bestIdxF));

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

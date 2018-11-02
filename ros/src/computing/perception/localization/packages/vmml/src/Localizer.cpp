/*
 * Localizer.cpp
 *
 *  Created on: Jul 23, 2018
 *      Author: sujiwo
 */

#include <Eigen/Eigen>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include "KeyFrame.h"
#include "Localizer.h"
#include "MapPoint.h"
#include "Optimizer.h"
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
	frame.sourceMap = sourceMap;
	frame.cameraParam = &this->localizerCamera;

	vector<kfid> placeCandidates = imgDb->findCandidates(frame);

	// for debugging
	vector<dataItemId> srcInfo(placeCandidates.size());
	for (int i=0; i<placeCandidates.size(); ++i) {
		srcInfo[i] = sourceMap->keyframe(placeCandidates[i])->getSourceItemId();
	}

	size_t bestKfScore=0;
	kfid bestKfId;
	vector<bool> isValidKfs (placeCandidates.size(), false);
	Pose currentFramePose;

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
//			debug_KF_F_Matching(kf, frame, mapPointMatches);

			// store index of inlier pairs here
			vector<int> inliers;

			solvePose(kf, frame, mapPointMatches, currentFramePose, &inliers);

			// Store the inliers
			for (auto iidx: inliers) {
				auto p = mapPointMatches.at(iidx);
				frame.vfMapPoints[p.first] = p.second;
			}

			// XXX: Call pose optimization
			optimize_pose(frame, currentFramePose, sourceMap);

			continue;
		}

	}

	return bestKfId;
}


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


bool
Localizer::solvePose (
	const KeyFrame &kf,
	const Frame &fr,
	const vector<pair<mpid,kpid>> &mapPtMatchPairs,
	Pose& frpose,
	vector<int> *inliers)
const
{
	// XXX: Use cv::solvePnPRansac()
	// XXX: Need to clarify utilization of the camera poses

	Eigen::Matrix4d eKfExt = kf.externalParamMatrix4();

	Eigen::Matrix3d eKfRotMat = eKfExt.block<3,3>(0,0);
	cv::Mat cKfRotMat, cRVec;
	cv::eigen2cv(eKfRotMat, cKfRotMat);
	cv::Rodrigues(cKfRotMat, cRVec);

	Eigen::Vector3d eKfTransVec = eKfExt.block<3,1>(0,3);
	cv::Mat cKfTransVec;
	cv::eigen2cv(eKfTransVec, cKfTransVec);

	cv::Mat
		objectPoints (mapPtMatchPairs.size(), 3, CV_32F),
		imagePoints (mapPtMatchPairs.size(), 2, CV_32F);
	for (int r=0; r<mapPtMatchPairs.size(); ++r) {
		objectPoints.at<float>(r, 0) = sourceMap->mappoint(mapPtMatchPairs[r].first)->X();
		objectPoints.at<float>(r, 1) = sourceMap->mappoint(mapPtMatchPairs[r].first)->Y();
		objectPoints.at<float>(r, 2) = sourceMap->mappoint(mapPtMatchPairs[r].first)->Z();
		imagePoints.at<float>(r, 0) = fr.keypoint(mapPtMatchPairs[r].second).pt.x;
		imagePoints.at<float>(r, 1) = fr.keypoint(mapPtMatchPairs[r].second).pt.y;
	}

	cv::Mat cameraMatrix = sourceMap->getCameraParameter(0).toCvMat();

	cv::Mat inlierIdx;

	bool hasSolution = cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, cv::Mat(), cRVec, cKfTransVec, true, 100, 4.0, 0.99, inlierIdx, cv::SOLVEPNP_EPNP);
	if (hasSolution==false)
		return false;

	cv::Rodrigues(cRVec, cKfRotMat);
	cv::cv2eigen(cKfRotMat, eKfRotMat);
	Eigen::Quaterniond Q;
	Q = eKfRotMat;
	cv::cv2eigen(cKfTransVec, eKfTransVec);

	frpose = Pose::from_Pos_Quat(eKfTransVec, Q).inverse();

	if (inliers != nullptr) {
		inliers->resize(inlierIdx.rows);
		for (int i=0; i<inlierIdx.rows; ++i) {
			inliers->at(i) = inlierIdx.at<int>(i,0);
		}
	}

	return true;
}

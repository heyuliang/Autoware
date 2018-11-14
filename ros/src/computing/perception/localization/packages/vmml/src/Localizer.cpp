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

#include <omp.h>


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

	lastGoodFrame = nullptr;
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
		const cv::KeyPoint k_in_kf = keyframe.keypoint(_kp);
		const cv::KeyPoint k_in_f = frame.keypoint(mpair.second);
		cv::line(img, k_in_kf.pt, k_in_f.pt, colorGreen, 1);
		cv::circle(img, k_in_kf.pt, 2, colorRed);
	}

	cv::imwrite(dpath, img);
}


bool
Localizer::detect (cv::Mat &frmImg, kfid &srcMapKfId, Pose &computedPose)
{
	// Should only be set during debugging session
	const bool debugMatching = false;
	ptime t1;

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
	frame.computeBoW(*imgDb);

	auto placeCandidates = imgDb->findCandidates(frame);

	// for debugging
	vector<dataItemId> srcInfo(placeCandidates.size());
	for (int i=0; i<placeCandidates.size(); ++i) {
		srcInfo[i] = sourceMap->keyframe(placeCandidates[i])->getSourceItemId();
	}

	size_t bestKfScore=0;
	vector<bool> isValidKfs (placeCandidates.size(), false);
	bool gotValidPose = false;
	Pose currentFramePose;

	int numCpu = omp_get_num_procs();

	for (int i=0; i<placeCandidates.size(); i++) {

		const KeyFrame &kf = *sourceMap->keyframe(placeCandidates[i]);
		/*
		 * Some notes on this vector:
		 * This is a pair of mappoint ID to its reflection (as keypoint) in
		 * currently investigated frame
		 */
		vector<pair<mpid,kpid>> mapPointMatches;

		t1 = getCurrentTime();
		/*
		 * Notes: this routine eats up the most significant runtime inside place detection.
		 * Solution:
		 * 1. Parallelization
		 * 2. ?
		 */
		int numMatches = SearchBoW (kf, frame, mapPointMatches);
		debugMsg("1: " + to_string(double((getCurrentTime() - t1).total_microseconds()) / 1e6));


		if (numMatches >= 15) {
			isValidKfs[i] = true;

			if (debugMatching==true)
				debug_KF_F_Matching(kf, frame, mapPointMatches);

			// store index of inlier pairs here
			vector<int> inliers;

			t1 = getCurrentTime();
			solvePose(kf, frame, mapPointMatches, currentFramePose, &inliers);
			debugMsg("2: " + to_string(double((getCurrentTime() - t1).total_microseconds()) / 1e6));

			if (debugMatching==true)
				debug_KF_F_Matching(kf, frame, mapPointMatches);

			// Store the inliers
			for (auto iidx: inliers) {
				auto p = mapPointMatches.at(iidx);
				frame.vfMapPoints[p.first] = p.second;
			}

			t1 = getCurrentTime();
			int numInliers = optimize_pose(frame, currentFramePose, sourceMap);
			debugMsg("3: " + to_string(double((getCurrentTime() - t1).total_microseconds()) / 1e6));

			if (numInliers >= numMatches/2 and gotValidPose==false)
				gotValidPose = true;

			srcMapKfId = kf.getId();
			continue;
		}
	}

	if (gotValidPose==true) {
		computedPose = currentFramePose;
	}

	return gotValidPose;
}


/*
 * Match the map points in kf to keypoints in frame
 */
int
Localizer::SearchBoW (const KeyFrame &kf, const Frame &frame, vector<pair<mpid,kpid>> &mapPtMatchPairs, const float matchNNRatio) const
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
			int vInxSz = vIndicesKF.size();

			for (uint32_t iKF=0; iKF<vIndicesKF.size(); iKF++) {

				auto realKpIdxKF = vIndicesKF[iKF];
				mpid nMpId;
				try {
					nMpId = keyPtMapPoint.at(realKpIdxKF);
				} catch (out_of_range &e) {
					continue;
				}

				const cv::Mat &descKf = kf.descriptor(realKpIdxKF);

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
						const cv::KeyPoint &kp = kf.keypoint(realKpIdxKF);
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


bool
Localizer::detect_mt (cv::Mat &frmImg, kfid &srcMapKfId, Pose &computedPose)
{
	// Should only be set during debugging session
	const bool debugMatching = false;
	ptime t1;

	cv::Mat rzImg;
	if (frmImg.cols != localizerCamera.width) {
		float ratio = float(localizerCamera.width) / float(frmImg.cols);
		cv::resize(frmImg, rzImg, cv::Size(), ratio, ratio, cv::INTER_CUBIC);
	}
	else
		rzImg = frmImg;

	auto frame = createFrame(rzImg);

	auto placeCandidates = imgDb->findCandidates(*frame);

	// for debugging
	vector<dataItemId> srcInfo(placeCandidates.size());
	for (int i=0; i<placeCandidates.size(); ++i) {
		srcInfo[i] = sourceMap->keyframe(placeCandidates[i])->getSourceItemId();
	}

	size_t bestKfScore=0;
	vector<bool> isValidKfs (placeCandidates.size(), false);
	bool gotValidPose = false;
	Pose currentFramePose;

	const int numCpu = omp_get_num_procs();
	const int numRepeatSearch = (placeCandidates.size() + numCpu - 1) / numCpu;
	vector<int> numValidMatches (numCpu, 0);
	vector < vector <pair <mpid,kpid> > > mapPointMatches (numCpu);

	for (int r=0; r<numRepeatSearch; r++) {

		int numProc;
		if (r<numRepeatSearch-1)
			numProc = numCpu;
		else
			numProc = placeCandidates.size() % numCpu;

		// Parallelizable
#pragma omp parallel for
		for (int c=0; c<numProc; c++) {

			int p = r*numCpu + c;
			const KeyFrame &kf = *sourceMap->keyframe(placeCandidates[p]);
			mapPointMatches[c].clear();
			numValidMatches[c] = SearchBoW(kf, *frame, mapPointMatches[c]);

		}

		// Join. (only using single thread here)
		vector<int> inliers;
		for (int c=0; c<numProc; c++) {

			if (numValidMatches[c] >= 15) {

				int p = r*numCpu + c;
				const KeyFrame &kf = *sourceMap->keyframe(placeCandidates[p]);

				inliers.clear();
				solvePose(kf, *frame, mapPointMatches[c], currentFramePose, &inliers);

				// Store the inliers
				for (auto iidx: inliers) {
					const auto &match = mapPointMatches[c].at(iidx);
					frame->vfMapPoints[match.first] = match.second;
				}

				int numInliers = optimize_pose(*frame, currentFramePose, sourceMap);
				if (numInliers >= numValidMatches[c]/2 and gotValidPose==false) {
					gotValidPose = true;
					srcMapKfId = kf.getId();
				}

				// XXX: Should we break off when pose is found ?

				continue;
			}
		}

		if (gotValidPose)
			break;

		continue;
	}

	if (gotValidPose) {
		computedPose = currentFramePose;
		lastGoodFrame = frame;
	}
	else {
		lastGoodFrame = nullptr;
	}
	return gotValidPose;
}


shared_ptr<Frame>
Localizer::createFrame (cv::Mat &imgSrc) const
{
	shared_ptr<Frame> newFrame (new Frame(imgSrc, this, &this->localizerCamera));
	newFrame->sourceMap = sourceMap;
	newFrame->computeBoW(*imgDb);

	return newFrame;
}


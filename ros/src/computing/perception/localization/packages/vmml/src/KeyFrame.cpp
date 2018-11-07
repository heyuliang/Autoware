/*
 * KeyFrame.cpp
 *
 *  Created on: Jun 5, 2018
 *      Author: sujiwo
 */

#include "KeyFrame.h"
#include <exception>
#include <bitset>
#include "Frame.h"
//#include "MapBuilder.h"
#include "utilities.h"
#include "MapPoint.h"



using namespace std;
using namespace Eigen;


/*
 * Ensure that KeyFrame ID is always positive and non-zero
 */
kfid KeyFrame::nextId = 1;


Vector2d cv2eigen (const cv::Point2f &p)
{ return Eigen::Vector2d (p.x, p.y); }

typedef Matrix<double,3,4> poseMatrix;
typedef Matrix4d poseMatrix4;


const float pixelReprojectionError = 6.0;


std::set<kpid>
KeyFrame::allKeyPointId (const KeyFrame &kf)
{
	std::set<kpid> allkp;
	for (kpid i=0; i<kf.fKeypoints.size(); i++)
		allkp.insert(i);
	return allkp;
}


KeyFrame::KeyFrame()
{}


KeyFrame::KeyFrame(
	const cv::Mat &imgSrc,
	const Vector3d &p, const Eigen::Quaterniond &o,
	cv::Mat &mask,
	cv::Ptr<cv::FeatureDetector> fdetector,
	const CameraPinholeParams *cameraIntr,
	const int _cameraId,
	dataItemId _srcItemId) :

	cameraId(_cameraId),
	frCreationTime(boost::posix_time::second_clock::local_time()),
	srcItemId(_srcItemId),
	parentMap(NULL)

{
	setPose(p, o);
	cameraParam = const_cast<CameraPinholeParams*>(cameraIntr);

	if(cameraIntr->width < 0 or cameraIntr->height < 0)
		throw runtime_error("Camera parameter has not been initialized properly (<0)");

	id = nextId++;

	// Enforce gray image before computing features
	cv::Mat grayImg;
	if (imgSrc.channels()==1)
		grayImg = imgSrc;
	else
		cv::cvtColor(imgSrc, grayImg, CV_BGR2GRAY, 1);
	image = grayImg;
	computeFeatures(fdetector, mask);
}


KeyFrame::~KeyFrame()
{
	// TODO Auto-generated destructor stub
}


void KeyFrame::match(const KeyFrame &k1, const KeyFrame &k2,
	cv::Ptr<cv::DescriptorMatcher> matcher,
	vector<FeaturePair> &featurePairs)
{
	vector<cv::DMatch> k12matches;
	matcher->match(k2.fDescriptors, k1.fDescriptors, k12matches);

	for (auto &m: k12matches) {
		if (m.trainIdx < k1.fKeypoints.size() and m.queryIdx < k2.fKeypoints.size()) {
			FeaturePair fp = {m.trainIdx, k1.fKeypoints[m.trainIdx].pt, m.queryIdx, k2.fKeypoints[m.queryIdx].pt};
			featurePairs.push_back (fp);
		}
	}
}


Eigen::Vector2f
convertToEigen (const cv::Point2f &P)
{
	return Eigen::Vector2f(P.x, P.y);
}



void debugMatch (const cv::Mat &imgToDraw, const vector<cv::DMatch> &matches, const vector<cv::KeyPoint> &kpList1, const vector<cv::KeyPoint> &kpList2, const string &filename)
{
	cv::Mat newColorImage;
	cv::cvtColor(imgToDraw, newColorImage, CV_GRAY2BGR);

	for (int i=0; i<matches.size(); ++i) {
		const cv::DMatch &mt = matches[i];
		if (mt.trainIdx >= kpList1.size() or mt.queryIdx >= kpList2.size())
			continue;
		const auto &kp1 = kpList1[mt.trainIdx];
		const auto &kp2 = kpList2[mt.queryIdx];
		cv::line(newColorImage, kp1.pt, kp2.pt, cv::Scalar(0,255,0));
	}

	cv::imwrite (filename, newColorImage);
}


/*
 * XXX: Observation results
 * false negatives due to missing projection
 */

void
KeyFrame::match (const KeyFrame &kf,
		const Frame &frame,
		std::vector<FeaturePair> &featurePairs,
		cv::Ptr<cv::DescriptorMatcher> matcher)
{
	static bool doDebugMatch = false;
	const int maxDebugMatchPair = 80;

	featurePairs.clear();

	// Matching itself
	vector<cv::DMatch> kf2fMatches;
	matcher->match(frame.fDescriptors, kf.fDescriptors, kf2fMatches);

	// Sort descending based on distance
	sort(kf2fMatches.begin(), kf2fMatches.end(),
		[&](const cv::DMatch &m1, const cv::DMatch &m2) -> bool
		{ return m1.distance < m2.distance; }
	);

	cv::Mat newColorImage;
	if (doDebugMatch) {
		cv::cvtColor(frame.getImage(), newColorImage, CV_GRAY2BGR);
	}

	// Projection check
	int correctProjection = 0;
	for (int i=0; i<kf2fMatches.size(); ++i) {
		auto &m = kf2fMatches[i];

		if (m.trainIdx >= kf.fKeypoints.size() or m.queryIdx >= frame.fKeypoints.size())
			continue;

		const cv::Point2f &frameKp = frame.fKeypoints[m.queryIdx].pt;

		if (doDebugMatch) {
			if (i<=maxDebugMatchPair)
			cv::line(newColorImage, kf.fKeypoints[m.trainIdx].pt, frameKp, cv::Scalar(0,255,0));
		}

		/*
		 * XXX: We need an implementation of semi optical flow here
		 */
		float projDev = (convertToEigen(frameKp) - convertToEigen(kf.fKeypoints[m.trainIdx].pt)).norm();
		if (projDev >= pixelReprojectionError)
			continue;

		FeaturePair fp = {m.trainIdx, kf.fKeypoints[m.trainIdx].pt, m.queryIdx, frame.fKeypoints[m.queryIdx].pt};
		featurePairs.push_back(fp);

	}

	if (doDebugMatch) {
		cv::imwrite ("/tmp/match-1237-f.png", newColorImage);
	}
}


void KeyFrame::matchSubset (
	const KeyFrame &k1, const KeyFrame &k2,
	cv::Ptr<cv::DescriptorMatcher> matcher,
	std::vector<FeaturePair> &featurePairs,
	const kpidField &kp1list, const kpidField &kp2list)
{
	assert (kp1list.size()==k1.fKeypoints.size());
	assert (kp2list.size()==k2.fKeypoints.size());

	cv::Mat mask = kpidField::createMask(kp1list, kp2list);
	vector<cv::DMatch> k12matches;
	matcher->match(k2.fDescriptors, k1.fDescriptors, k12matches, mask);

	for (auto &m: k12matches) {
		if (m.trainIdx < k1.fKeypoints.size() and m.queryIdx < k2.fKeypoints.size()) {
			FeaturePair fp = {m.trainIdx, k1.fKeypoints[m.trainIdx].pt, m.queryIdx, k2.fKeypoints[m.queryIdx].pt};
			featurePairs.push_back (fp);
		}
	}
}



void KeyFrame::triangulate (
	const KeyFrame *kf1, const KeyFrame *kf2,
	std::vector<mpid> &mapPointList,
	const std::vector<FeaturePair> &featurePairs,
	std::map<mpid, kpid> &mapPointToKeyPointInKeyFrame1,
	std::map<mpid, kpid> &mapPointToKeyPointInKeyFrame2,
	VMap *parent)
{
	set<uint> badMatches;

	const poseMatrix pm1 = kf1->projectionMatrix(),
		pm2 = kf2->projectionMatrix();

	mapPointList.clear();

	for (uint i=0; i<featurePairs.size(); i++) {
		auto &fp = featurePairs[i];
		Vector2d proj1 = cv2eigen(fp.keypoint1),
			proj2 = cv2eigen(fp.keypoint2);

		Vector4d triangulatedpt;
		TriangulateDLT (pm1, pm2, proj1, proj2, triangulatedpt);
		Vector3d pointm = triangulatedpt.head(3);

		// Check for Reprojection Errors
		float pj1 = (kf1->project(pointm) - proj1).norm(),
			pj2 = (kf2->project(pointm) - proj2).norm();
		if (pj1 > pixelReprojectionError or pj2 > pixelReprojectionError)
			continue;

		// checking for regularity of triangulation result
		// 1: Point must be in front of camera
		Vector3d v1 = pointm - kf1->position();
		double cos1 = v1.dot(kf1->normal()) / v1.norm();
		if (cos1 < 0)
			continue;
		double dist1 = v1.norm();
		Vector3d v2 = pointm - kf2->position();
		double cos2 = v2.dot(kf2->normal()) / v2.norm();
		if (cos2 < 0)
			continue;
		double dist2 = v2.norm();

		// 2: Must have enough parallax (ie. remove faraway points)
		double cosParallax = (-v1).dot(-v2) / (dist1 * dist2);
		if (cosParallax >= 0.999990481)
			continue;

		mpid newMp = parent->createMapPoint(pointm);
		mapPointList.push_back(newMp);
		mapPointToKeyPointInKeyFrame1[newMp] = fp.kpid1;
		mapPointToKeyPointInKeyFrame2[newMp] = fp.kpid2;
	}
}


Eigen::Vector2d
KeyFrame::project (const MapPoint &pt3) const
{
	return project(pt3.getPosition());
}


vector<Vector2d>
KeyFrame::projectAllMapPoints() const
{
	auto visibleMapPts = parentMap->allMapPointsAtKeyFrame(this->id);
	vector<Vector2d> projectionResult(visibleMapPts.size());

	int i = 0;
	for (auto &ptx: visibleMapPts) {
		mpid pt3d = ptx.first;
		projectionResult[i] = project(*parentMap->mappoint(pt3d));
		i++;
	}

	return projectionResult;
}


void
KeyFrame::debugMapPoints() const
{
	static const string _dbgKfMapPoints = "/tmp/kf_mappoints";
}


void
KeyFrame::debugKeyPoints() const
{

}

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


kfid KeyFrame::nextId = 0;


Vector2d cv2eigen (const cv::Point2f &p)
{ return Eigen::Vector2d (p.x, p.y); }

typedef Matrix<double,3,4> poseMatrix;
typedef Matrix4d poseMatrix4;


#define pixelReprojectionError 6.0


std::set<kpid>
KeyFrame::allKeyPointId (const KeyFrame &kf)
{
	std::set<kpid> allkp;
	for (kpid i=0; i<kf.keypoints.size(); i++)
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

	orientation(o),
	position(p),
	cameraId(_cameraId),
	frCreationTime(boost::posix_time::second_clock::local_time()),
	srcItemId(_srcItemId),
	parentMap(NULL)

{
	if(cameraIntr->width < 0 or cameraIntr->height < 0)
		throw runtime_error("Camera parameter has not been initialized properly (<0)");

	id = nextId++;

	normal = externalParamMatrix().block(0,0,3,3).transpose().col(2);

	// Enforce gray image before computing features
	cv::Mat grayImg;
	if (imgSrc.channels()==1)
		grayImg = imgSrc;
	else
		cv::cvtColor(imgSrc, grayImg, CV_BGR2GRAY, 1);
	fdetector->detectAndCompute(grayImg, mask, keypoints, descriptors, false);
	image = grayImg;

	Matrix<double,3,4> camInt = cameraIntr->toMatrix();
	projMatrix = cameraIntr->toMatrix() * externalParamMatrix4();
}


KeyFrame::~KeyFrame()
{
	// TODO Auto-generated destructor stub
}


// XXX: 3x4 or 4x4 ?
poseMatrix KeyFrame::externalParamMatrix () const
{
	poseMatrix ex = poseMatrix::Zero();
	Matrix3d R = orientation.toRotationMatrix().transpose();
	ex.block<3,3>(0,0) = R;
	ex.col(3) = -(R*position);
	return ex;
}


poseMatrix4 KeyFrame::externalParamMatrix4() const
{
	poseMatrix4 ex = poseMatrix4::Identity();
	Matrix3d R = orientation.toRotationMatrix().transpose();
	ex.block<3,3>(0,0) = R;
	ex.col(3).head(3) = -(R*position);
	return ex;
}


void KeyFrame::match(const KeyFrame &k1, const KeyFrame &k2,
	cv::Ptr<cv::DescriptorMatcher> matcher,
	vector<FeaturePair> &featurePairs)
{
	vector<cv::DMatch> k12matches;
	matcher->match(k2.descriptors, k1.descriptors, k12matches);

	for (auto &m: k12matches) {
		if (m.trainIdx < k1.keypoints.size() and m.queryIdx < k2.keypoints.size()) {
			FeaturePair fp = {m.trainIdx, k1.keypoints[m.trainIdx].pt, m.queryIdx, k2.keypoints[m.queryIdx].pt};
			featurePairs.push_back (fp);
		}
	}
}


Eigen::Vector2f
convertToEigen (const cv::Point2f &P)
{
	return Eigen::Vector2f(P.x, P.y);
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
	// Select all map points and their keypoints in the keyframe
	auto mapPtList = kf.parentMap->allMapPointsAtKeyFrame(kf.id);
	cv::Mat kfMpDescriptors (mapPtList.size(), kf.descriptors.cols, kf.descriptors.type());
	vector<kpid> kpToMatchList (mapPtList.size());
	featurePairs.clear();

	int i = 0;
	for (auto &p: mapPtList) {
		kfMpDescriptors.row(i) = kf.getDescriptorAt(p.second);
		kpToMatchList[i] = p.second;
		i++;
	}

	// Matching itself
	vector<cv::DMatch> kf2fMatches;
	matcher->match(kfMpDescriptors, frame.descriptor(), kf2fMatches);

	// Projection check
	int correctProjection = 0;
	for (i=0; i<kf2fMatches.size(); ++i) {
		auto &m = kf2fMatches[i];

		if (m.trainIdx >= mapPtList.size() or m.queryIdx >= frame.keypoints.size())
			continue;

		kpid keyframeKp = kpToMatchList[m.trainIdx];
		const MapPoint &mp = *kf.parentMap->mappoint(kf.parentMap->getMapPointByKeypoint(kf.id, keyframeKp));
		const Vector2d keypointProj = kf.project(mp);
		const cv::Point2f &frameKp = frame.keypoints[m.queryIdx].pt;

		float projDev = (convertToEigen(frameKp) - keypointProj.cast<float>()).norm();
		if (projDev >= pixelReprojectionError)
			continue;

		FeaturePair fp = {keyframeKp, kf.keypoints[keyframeKp].pt, m.queryIdx, frame.keypoints[m.queryIdx].pt};
		featurePairs.push_back(fp);

	}
}


void KeyFrame::matchSubset (
	const KeyFrame &k1, const KeyFrame &k2,
	cv::Ptr<cv::DescriptorMatcher> matcher,
	std::vector<FeaturePair> &featurePairs,
	const kpidField &kp1list, const kpidField &kp2list)
{
	assert (kp1list.size()==k1.keypoints.size());
	assert (kp2list.size()==k2.keypoints.size());

	cv::Mat mask = kpidField::createMask(kp1list, kp2list);
	vector<cv::DMatch> k12matches;
	matcher->match(k2.descriptors, k1.descriptors, k12matches, mask);

	for (auto &m: k12matches) {
		if (m.trainIdx < k1.keypoints.size() and m.queryIdx < k2.keypoints.size()) {
			FeaturePair fp = {m.trainIdx, k1.keypoints[m.trainIdx].pt, m.queryIdx, k2.keypoints[m.queryIdx].pt};
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

	const poseMatrix &pm1 = kf1->projMatrix,
		&pm2 = kf2->projMatrix;

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
		Vector3d v1 = pointm - kf1->position;
		double cos1 = v1.dot(kf1->normal) / v1.norm();
		if (cos1 < 0)
			continue;
		double dist1 = v1.norm();
		Vector3d v2 = pointm - kf2->position;
		double cos2 = v2.dot(kf2->normal) / v2.norm();
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


Vector2d KeyFrame::project(const Vector3d &pt3) const
{
	Vector3d ptx = projMatrix * pt3.homogeneous();
	return ptx.head(2) / ptx[2];
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


Eigen::Vector3d
KeyFrame::transform (const Eigen::Vector3d &pt3) const
{
	Vector4d ptx = externalParamMatrix4()* pt3.homogeneous();
	return ptx.hnormalized();
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

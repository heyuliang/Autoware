/*
 * KeyFrame.h
 *
 *  Created on: Jun 5, 2018
 *      Author: sujiwo
 */

#ifndef KEYFRAME_H_
#define KEYFRAME_H_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <map>
#include <Eigen/Eigen>
#include <string>
#include <memory>
#include <tuple>
#include <boost/serialization/serialization.hpp>
#include <limits>

#include "VMap.h"
#include "MapPoint.h"
#include "triangulation.h"
#include "utilities.h"


namespace boost {
namespace serialization {
	template <class Archive>
		void serialize (Archive & ar, KeyFrame &keyframe, const unsigned int version);
}
}


//typedef std::tuple<int64, cv::Point2f, int64, cv::Point2f> FeaturePair;
struct FeaturePair {
	kpid kpid1;
	cv::Point2f keypoint1;
	kpid kpid2;
	cv::Point2f keypoint2;

	Eigen::Vector2d toEigen1 () const
	{ return Eigen::Vector2d(keypoint1.x, keypoint1.y); }

	Eigen::Vector2d toEigen2 () const
	{ return Eigen::Vector2d(keypoint2.x, keypoint2.y); }
};

struct CameraPinholeParams;

//const kfid defaultKfId = std::numeric_limits<kfid>;


class KeyFrame {
public:

	KeyFrame();

	KeyFrame(const cv::Mat &imgSrc,
			const Eigen::Vector3d &p, const Eigen::Quaterniond &o,
			cv::Mat &mask,
			cv::Ptr<cv::FeatureDetector> fdetector,
			const CameraPinholeParams *cameraIntr,
			const int _cameraId,
			kfid forceId=std::numeric_limits<kfid>::max());
	virtual ~KeyFrame();

	inline int numOfKeyPoints() const
	{ return keypoints.size(); }

	inline std::vector<cv::KeyPoint> getKeypoints() const
	{ return keypoints; }

	inline cv::KeyPoint getKeyPointAt (int idx) const
	{ return keypoints[idx]; }

	inline cv::Mat getDescriptors() const
	{ return descriptors; }

	inline cv::Mat getDescriptorAt(int idx) const
	{ return descriptors.row(idx).clone(); }

	static void match (const KeyFrame &k1, const KeyFrame &k2,
		cv::Ptr<cv::DescriptorMatcher> matcher,
		std::vector<FeaturePair> &featurePairs
	);

	static void matchSubset (
		const KeyFrame &k1, const KeyFrame &k2,
		cv::Ptr<cv::DescriptorMatcher> matcher,
		std::vector<FeaturePair> &featurePairs,
		const kpidField &kp1list, const kpidField &kp2list
	);

	static void triangulate (
		const KeyFrame *kf1, const KeyFrame *kf2,
		std::vector<kfid> &mapPointList,
		const std::vector<FeaturePair> &featurePairs,
		std::map<mpid, kpid> &mapPointToKeyPointInKeyFrame1,
		std::map<mpid, kpid> &mapPointToKeyPointInKeyFrame2,
		VMap *parent
	);

	kfid getId () const
	{ return id; }

	int getCameraId() const
	{ return cameraId; }

	Eigen::Matrix<double,3,4> externalParamMatrix () const;
	Eigen::Matrix4d externalParamMatrix4 () const;

	Eigen::Matrix<double,3,4> projectionMatrix () const
	{ return projMatrix; }

	// Project to 2D
	Eigen::Vector2d project (const Eigen::Vector3d &pt3) const;

	// Transform point (in world) to camera coordinate system
	Eigen::Vector3d transform (const Eigen::Vector3d &pt3) const;

	Eigen::Vector3d &getPosition ()
	{ return position; }

	Eigen::Quaterniond &getOrientation ()
	{ return orientation; }

	static std::set<kpid> allKeyPointId (const KeyFrame &kf);

//	Transform3d toEigen()
//	{ return Transform3d::fromPositionOrientationScale(Eigen::Translation3d(position), orientation, Eigen::Scaling(1.0)); }


protected:

	template <class Archive>
    friend void boost::serialization::serialize (Archive & ar, KeyFrame &keyframe, const unsigned int version);

	kfid id;
	cv::Mat image;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;

	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;
	Eigen::Vector3d normal;
	Eigen::Matrix<double,3,4> projMatrix;

	int cameraId;

	static kfid nextId;

//	KeyFrame* prev;
};

#endif /* KEYFRAME_H_ */

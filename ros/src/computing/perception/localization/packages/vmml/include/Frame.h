/*
 * Frame.h
 *
 *  Created on: Jul 17, 2018
 *      Author: sujiwo
 */

#ifndef FRAME_H_
#define FRAME_H_


#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "VMap.h"
#include "BaseFrame.h"
#include "utilities.h"

#include "DBoW2/BowVector.h"
#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"


class ImageDatabase;
class Localizer;


class Frame
{
public:
	friend class KeyFrame;

	Frame(cv::Mat &imgSrc,
		const Localizer* parent);
	virtual ~Frame();

	void computeBoW (const ImageDatabase &idb);

	const DBoW2::BowVector& getWords() const
	{ return words; }

	DBoW2::FeatureVector& getFeatureVector()
	{ return featureVec; }

	const cv::Mat& descriptor() const
	{ return mDescriptors; }

	cv::Mat descriptor(const int r)
	{ return mDescriptors.row(r); }

	const cv::KeyPoint &keypoint(const kpid kp) const
	{ return keypoints.at(kp); }

	const vector<cv::KeyPoint> &getAllKeyPoints() const
	{ return keypoints; }

	void debugKeyPoints () const;

	cv::Mat getImage() const
	{ return image.clone(); }

	Eigen::Vector3d &position ()
	{ return mPosition; }

	const Eigen::Vector3d &getPosition () const
	{ return mPosition; }

	Eigen::Quaterniond &orientation ()
	{ return mOrientation; }

	void setPose (const Pose &sp);

	Pose getPose () const
	{ return Pose::from_Pos_Quat(mPosition, mOrientation); }

	const Eigen::Quaterniond &getOrientation () const
	{ return mOrientation; }

protected:
	cv::Mat image;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat mDescriptors;

	Eigen::Vector3d mPosition;
	Eigen::Quaterniond mOrientation;

	DBoW2::BowVector words;
	DBoW2::FeatureVector featureVec;

};

#endif /* FRAME_H_ */

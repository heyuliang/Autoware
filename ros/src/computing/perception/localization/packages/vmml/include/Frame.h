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

#include "DBoW2/BowVector.h"
#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"


class ImageDatabase;
class Localizer;


class Frame
{
public:
	Frame(cv::Mat &imgSrc,
		const Localizer* parent);
	virtual ~Frame();

	void computeBoW (const ImageDatabase &idb);

	const DBoW2::BowVector& getWords() const
	{ return words; }

protected:
	cv::Mat image;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;

	Eigen::Vector3d _mPos;
	Eigen::Quaterniond _mOri;

	DBoW2::BowVector words;
	DBoW2::FeatureVector featureVec;
};

#endif /* FRAME_H_ */

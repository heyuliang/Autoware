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


class Frame : public BaseFrame
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

	void debugKeyPoints () const;

protected:

	DBoW2::BowVector words;
	DBoW2::FeatureVector featureVec;

};

#endif /* FRAME_H_ */

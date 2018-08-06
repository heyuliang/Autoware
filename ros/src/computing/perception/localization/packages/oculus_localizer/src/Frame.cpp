/*
 * Frame.cpp
 *
 *  Created on: Jul 17, 2018
 *      Author: sujiwo
 */

#include <vector>
#include "Frame.h"
#include "ImageDatabase.h"
#include "Localizer.h"


using namespace std;
using namespace Eigen;


Frame::	Frame(
	cv::Mat &imgSrc,
	const Localizer* parent
) :

	image(imgSrc),
	_mPos(Vector3d::Zero()),
	_mOri(Quaterniond::Identity())

{
	parent->getFeatureDetector()->detectAndCompute(
		image,
		// XXX: get a proper mask
		cv::Mat(),
		keypoints,
		descriptors,
		false);
}


Frame::~Frame() {
	// TODO Auto-generated destructor stub
}


void
Frame::computeBoW(const ImageDatabase &idb)
{
	if (words.empty()) {
		vector<cv::Mat> descWrd = toDescriptorVector(descriptors);
		idb.vocabulary().transform(descWrd, words, featureVec, 4);
	}
}

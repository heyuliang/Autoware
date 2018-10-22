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

#include "opencv2/opencv.hpp"
#include <boost/filesystem.hpp>


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
		parent->getMask(),
		keypoints,
		mDescriptors,
		false);
}


Frame::~Frame() {
	// TODO Auto-generated destructor stub
}


void
Frame::computeBoW(const ImageDatabase &idb)
{
	if (words.empty()) {
		vector<cv::Mat> descWrd = toDescriptorVector(mDescriptors);
		idb.vocabulary().transform(descWrd, words, featureVec, 4);
	}
}


const string _frameKpOutputName = "/tmp/frame_keypoints_descriptors.yml";


void
Frame::debugKeyPoints () const
{
	cv::FileStorage dbgFd (_frameKpOutputName, cv::FileStorage::Mode::WRITE);

	if (mDescriptors.rows != keypoints.size())
		throw runtime_error("#keypoints and descriptors are not equal");

	cv::Mat dKeypoints (keypoints.size(), 2, CV_32S);
	for (int i=0; i<keypoints.size(); i++) {
		dKeypoints.at<int>(i,0) = keypoints[i].pt.x;
		dKeypoints.at<int>(i,1) = keypoints[i].pt.y;
	}

	dbgFd << "Keypoints" << dKeypoints;
	dbgFd << "Descriptors" << this->mDescriptors;
}

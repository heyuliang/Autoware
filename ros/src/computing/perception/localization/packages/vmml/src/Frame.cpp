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


Frame::Frame(
	cv::Mat &imgSrc,
	const Localizer* parent
) :
	BaseFrame()
{
	/*parent->getFeatureDetector()->detectAndCompute(
		image,
		parent->getMask(),
		keypoints,
		mDescriptors,
		false);*/
	image = imgSrc;
	computeFeatures(parent->getFeatureDetector(), parent->getMask());
}


Frame::~Frame() {
	// TODO Auto-generated destructor stub
}


void
Frame::computeBoW(const ImageDatabase &idb)
{
	if (words.empty()) {
		vector<cv::Mat> descWrd = toDescriptorVector(fDescriptors);
		idb.vocabulary().transform(descWrd, words, featureVec, 4);
	}
}


const string _frameKpOutputName = "/tmp/frame_keypoints_descriptors.yml";


void
Frame::debugKeyPoints () const
{
	cv::FileStorage dbgFd (_frameKpOutputName, cv::FileStorage::Mode::WRITE);

	if (fDescriptors.rows != fKeypoints.size())
		throw runtime_error("#keypoints and descriptors are not equal");

	cv::Mat dKeypoints (fKeypoints.size(), 2, CV_32S);
	for (int i=0; i<fKeypoints.size(); i++) {
		dKeypoints.at<int>(i,0) = fKeypoints[i].pt.x;
		dKeypoints.at<int>(i,1) = fKeypoints[i].pt.y;
	}

	dbgFd << "Keypoints" << dKeypoints;
	dbgFd << "Descriptors" << this->fDescriptors;
}

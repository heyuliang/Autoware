/*
 * Localizer.h
 *
 *  Created on: Jul 23, 2018
 *      Author: sujiwo
 */

#ifndef LOCALIZER_H_
#define LOCALIZER_H_


#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include "VMap.h"
#include "optimizer.h"
#include "Frame.h"
#include "ImageDatabase.h"


class Localizer
{
public:

	Localizer(VMap*, bool emptyMask=false);

	virtual ~Localizer();

	kfid detect (cv::Mat &frmImg);

	void setCameraParameter (const CameraPinholeParams &c)
	{ localizerCamera = c; }

	void setCameraParameterFromId (int cameraId);

	const CameraPinholeParams& getCamera() const
	{ return localizerCamera; }

	void setMask(const cv::Mat &mm)
	{ locMask = mm.clone(); }

	cv::Mat getMask() const
	{ return locMask; }

	cv::Ptr<cv::FeatureDetector> getFeatureDetector() const
	{ return featureDetector; }

protected:
	VMap *sourceMap;
	ImageDatabase *imgDb;

	cv::Ptr<cv::FeatureDetector> featureDetector;

	CameraPinholeParams localizerCamera;

	cv::Mat locMask;
};

#endif /* LOCALIZER_H_ */

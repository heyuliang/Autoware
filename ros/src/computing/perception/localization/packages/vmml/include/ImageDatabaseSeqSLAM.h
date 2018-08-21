/*
 * ImageDatabaseSeqSLAM.h
 *
 *  Created on: Aug 20, 2018
 *      Author: sujiwo
 *
 *  An Implementation of SeqSLAM
 *  Credit To: Saburo Okita
 */

#ifndef _IMAGEDATABASESEQSLAM_H_
#define _IMAGEDATABASESEQSLAM_H_

#include <vector>
#include <opencv2/opencv.hpp>

#include "datasets/GenericDataset.h"


class ImageDatabaseSeqSLAM
{
public:
	ImageDatabaseSeqSLAM();
	virtual ~ImageDatabaseSeqSLAM();

	void learn (const cv::Mat &imgsrc);
	void learn (const std::vector<cv::Mat> &);

	void find (const cv::Mat &f);

protected:
    int patchSize = 8;
    int localRadius = 10;
    int matchingDistance = 10;
    int RWindow = 10;
    float minVelocity = 0.8;
    float maxVelocity = 1.2;

private:
    static cv::Mat normalizePatch (const cv::Mat &src, int patch_size);
};

#endif /* _IMAGEDATABASESEQSLAM_H_ */

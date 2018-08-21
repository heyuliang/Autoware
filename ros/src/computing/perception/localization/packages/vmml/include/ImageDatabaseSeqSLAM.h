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
#include <Eigen/Core>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include "cvobj_serialization.h"
#include "datasets/GenericDataset.h"


class ImageDatabaseSeqSLAM
{
public:
	ImageDatabaseSeqSLAM();
	virtual ~ImageDatabaseSeqSLAM();

	void learn (const cv::Mat &imgsrc);
//	void learn (const std::vector<cv::Mat> &);

	void build ();

	void find (const cv::Mat &f);

	static cv::Mat normalizePatch (const cv::Mat &src, int patch_size);

	template<class Archive>
	inline void serialize(Archive &ar, const unsigned int version)
	{
		ar & learntNormalizedImages;
	}

protected:
    int patchSize = 8;
    int localRadius = 10;
    int matchingDistance = 10;
    int RWindow = 10;
    float minVelocity = 0.8;
    float maxVelocity = 1.2;

private:
    std::vector<cv::Mat> learntNormalizedImages;

private:
    Eigen::VectorXd calculateDifferenceEnhancedVector (const cv::Mat &f) const;

};

#endif /* _IMAGEDATABASESEQSLAM_H_ */

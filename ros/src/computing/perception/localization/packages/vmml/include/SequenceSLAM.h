/*
 * ImageDatabaseSeqSLAM.h
 *
 *  Created on: Aug 20, 2018
 *      Author: sujiwo
 *
 *  An Implementation of SeqSLAM
 *  Credit To: Saburo Okita
 */

#ifndef _SEQSLAM_H_
#define _SEQSLAM_H_

#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include "cvobj_serialization.h"
#include "datasets/GenericDataset.h"


class SequenceSLAM
{
public:

	static const dataItemId notId=std::numeric_limits<dataItemId>::max();

	SequenceSLAM();
	virtual ~SequenceSLAM();

	void learn (const cv::Mat &imgsrc, dataItemId frameId=notId);
//	void learn (const std::vector<cv::Mat> &);

	void build ();

	void find (const cv::Mat &f, const int matchingDistance) const;

	/*
	 * XXX: all input images must have same size with training images
	 */
	void find (const std::vector<cv::Mat> &imgLst, const int matchingDistance) const;

	static cv::Mat normalizePatch (const cv::Mat &src, int patch_size);

	template<class Archive>
	inline void serialize(Archive &ar, const unsigned int version)
	{
		ar & learntNormalizedImages;
		ar & dataSetMap;
	}

	void imageSize(int &width, int &height) const
	{ width=learntNormalizedImages[0].cols, height=learntNormalizedImages[0].rows; }

protected:
    int patchSize = 8;
    int localRadius = 10;
    int matchingDistance = 10;
    int RWindow = 10;
    float minVelocity = 0.8;
    float maxVelocity = 1.2;

private:
    std::vector<cv::Mat> learntNormalizedImages;

    std::map<int, dataItemId> dataSetMap;

private:
//    Eigen::VectorXd calculateDifferenceEnhancedMatrix (const cv::Mat &f) const;

    Eigen::MatrixXd calculateDifferenceEnhancedMatrix (const std::vector<cv::Mat> &preprocSrc) const;

    std::pair<int,double> findMatch(const Eigen::MatrixXd &diffMat, const int N, const int m_dist) const;
};

#endif /* _SEQSLAM_H_ */

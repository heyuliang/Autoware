/*
 * ImageDatabaseSeqSLAM.cpp
 *
 *  Created on: Aug 20, 2018
 *      Author: sujiwo
 */

#include <ImageDatabaseSeqSLAM.h>


ImageDatabaseSeqSLAM::ImageDatabaseSeqSLAM()
{
	// TODO Auto-generated constructor stub
}


ImageDatabaseSeqSLAM::~ImageDatabaseSeqSLAM()
{
	// TODO Auto-generated destructor stub
}


void
ImageDatabaseSeqSLAM::learn (const cv::Mat &imgsrc)
{
	cv::Mat preprocessedImg = normalizePatch(imgsrc, patchSize);
	learntNormalizedImages.push_back(preprocessedImg);
}


cv::Mat
ImageDatabaseSeqSLAM::normalizePatch (const cv::Mat &src, int patch_size)
{
	assert((src.cols%patch_size==0) and (src.rows%patch_size==0));
	assert(src.channels()==1);

	cv::Mat result = src.clone();
	cv::Mat patch, patch_mean, patch_stddev, temp;
	int patch_area = patch_size*patch_size;

	for (int y=0; y<result.rows; y+=patch_size) {
		for (int x=0; x<result.cols; x+=patch_size) {

			patch = cv::Mat(result, cv::Rect(x, y, patch_size, patch_size));
			cv::meanStdDev(patch, patch_mean, patch_stddev);
			float mean_val = patch_mean.at<float>(0, 0);
			float stddev_val = sqrt((pow(patch_stddev.at<float>(0,0), 2) * patch_area) / (patch_area-1));

			patch.convertTo(temp, CV_32FC1);

			if (stddev_val > 0.0) {
				for (cv::MatIterator_<float>it=temp.begin<float>(); it!=temp.end<float>(); ++it) {
					*it = 127 + cvRound((*it - mean_val) / stddev_val);
				}
			}
			else {
				temp = cv::Scalar::all(0);
			}

			temp.convertTo(patch, CV_8UC1);
		}
	}

	return result;
}


void
ImageDatabaseSeqSLAM::find(const cv::Mat &frame)
{
	Eigen::VectorXd difvec = calculateDifferenceEnhancedVector(frame);
}


Eigen::VectorXd
ImageDatabaseSeqSLAM::calculateDifferenceEnhancedVector (const cv::Mat &frame) const
{
	const uint32_t N = learntNormalizedImages.size();
	Eigen::VectorXd diffVec (N+1);

	for (int i=0; i<N; i++) {
		double S = cv::sum(cv::abs(frame-learntNormalizedImages[i]))[0] / N;
		diffVec[i] = S;
	}
	/* Includes additional value on diff vector with infinite value, to penalize out of bounds cases */
	diffVec[N] = std::numeric_limits<double>::max();

	// Enhance local contrast


	return diffVec;
}

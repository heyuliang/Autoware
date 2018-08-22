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
	Eigen::VectorXd diffVec = calculateDifferenceEnhancedVector(frame);

	// findMatches

	Eigen::VectorXd matches =
		Eigen::VectorXd::Constant(matchingDistance/2,
			std::numeric_limits<Eigen::VectorXd::Scalar>::max());

	// XXX: Unfinished
}


template<typename Scalar, int numRows>
void meanStdDev (const Eigen::Matrix<Scalar,numRows,1> &V, Scalar &mean, Scalar &stddev, bool sampleStdDev=false)
{
	mean = V.mean();

	Eigen::Matrix<Scalar,numRows,1> C;
	for (int i=0; i<numRows; i++) {
		Scalar s = V(i,0) - mean;
		C(i,0) = s*s;
	}
	if (sampleStdDev==true)
		stddev = sqrt(C.sum() / (numRows-1));
	else
		stddev = sqrt(C.sum() / numRows);
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
	Eigen::VectorXd diffEnhanced (diffVec.size());
	for (int i=0; i<diffVec.size(); i++) {
		int lowerBound = std::max(0, i-localRadius/2);
		int upperBound = std::min((int)diffVec.size(), i+1+localRadius/2);

		double mean, stddev;
		Eigen::VectorXd patch = diffVec.block(lowerBound, 0, upperBound-lowerBound, 1);
		meanStdDev(patch, mean, stddev, true);

		/* Enhance contrast by (local_patch - patch_mean) / patch_stddev */
		diffEnhanced[i] = (diffVec[i] - mean) * (1/stddev);
	}

	/* Shift so that the minimum value in the vector is 0 */
	double minval = diffEnhanced.minCoeff();
	for (int i=0; i<diffEnhanced.size(); i++)
		diffEnhanced[i] -= minval;

	return diffEnhanced;
}

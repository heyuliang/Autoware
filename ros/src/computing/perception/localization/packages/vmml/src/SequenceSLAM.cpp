/*
 * ImageDatabaseSeqSLAM.cpp
 *
 *  Created on: Aug 20, 2018
 *      Author: sujiwo
 */

#include <SequenceSLAM.h>


using namespace std;
using namespace Eigen;


SequenceSLAM::SequenceSLAM()
{
	// TODO Auto-generated constructor stub
}


SequenceSLAM::~SequenceSLAM()
{
	// TODO Auto-generated destructor stub
}


void
SequenceSLAM::learn (const cv::Mat &imgsrc)
{
	cv::Mat preprocessedImg = normalizePatch(imgsrc, patchSize);
	learntNormalizedImages.push_back(preprocessedImg);
}


cv::Mat
SequenceSLAM::normalizePatch (const cv::Mat &src, int patch_size)
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
SequenceSLAM::find(const cv::Mat &frame)
{
	Eigen::VectorXd diffVec = calculateDifferenceEnhancedVector(frame);

	// findMatches

	Eigen::VectorXd matches =
		Eigen::VectorXd::Constant(matchingDistance/2,
			std::numeric_limits<Eigen::VectorXd::Scalar>::max());

	// XXX: Unfinished
}


void
SequenceSLAM::find (const vector<cv::Mat> &imgLst, const int matching_distance)
const
{
	// Preprocess input images
	vector<cv::Mat> preprocessSrc(imgLst.size());
	for (int i=0; i<imgLst.size(); i++) {
		preprocessSrc.at(i) = normalizePatch(imgLst[i], patchSize);
	}

	MatrixXd diffEnhancedMat = calculateDifferenceEnhancedVector(preprocessSrc);

	int mDist = matching_distance + (matching_distance % 2);
	int half_mDist = mDist / 2;

	MatrixXf matches =
		MatrixXf::Constant(2, diffEnhancedMat.cols(),
			std::numeric_limits<MatrixXf::Scalar>::max());

	for (int N=half_mDist+1; N<(diffEnhancedMat.cols() - half_mDist); N++) {
		pair<int,double> match = findMatch(diffEnhancedMat, N, mDist);
		matches(0, N) = match.first;
		matches(1, N) = match.second;
	}
}


std::pair<int,double>
SequenceSLAM::findMatch
(const Eigen::MatrixXd &diffMat, const int N, const int m_dist)
{
	// XXX: Unfinished
}

//template<typename Scalar, const int numRows>
//void meanStdDev (const Eigen::Matrix<Scalar,numRows,1> &V, Scalar &mean, Scalar &stddev, bool sampleStdDev=false)
//{
//	mean = V.mean();
//
//	Scalar accum = 0;
//	for (int i=0; i<numRows; i++) {
//		Scalar s = V[i] - mean;
//		accum += s*s;
//	}
//	if (sampleStdDev==true)
//		stddev = sqrt(accum / (numRows-1));
//	else
//		stddev = sqrt(accum / numRows);
//}


template<typename Derived, typename Scalar>
void meanStdDev (
	const Eigen::MatrixBase<Derived> &V,
	Scalar &mean, Scalar &stddev,
	bool sampleStdDev=false)
{
	const int N = V.rows() * V.cols();
//	mean = V.cwiseAbs().sum() / N;
	mean = V.sum() / N;

	Scalar accum = 0;
	for (int i=0; i<N; i++) {
		Scalar s = V[i] - mean;
		accum += s*s;
	}

	if (sampleStdDev==true)
		stddev = sqrt(accum / (N-1));
	else
		stddev = sqrt(accum / N);
}


Eigen::VectorXd
SequenceSLAM::calculateDifferenceEnhancedVector (const cv::Mat &frame) const
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


MatrixXd
SequenceSLAM::calculateDifferenceEnhancedVector (const std::vector<cv::Mat> &preprocSrc)
const
{
	const uint32_t N = learntNormalizedImages.size(),
		cols = preprocSrc.size();

	MatrixXd diffMat = MatrixXd::Zero(N+1, preprocSrc.size());

	for (int i=0; i<N; i++) {
		for (int j=0; j<preprocSrc.size(); j++) {
			double S = cv::sum(cv::abs(preprocSrc[j]-learntNormalizedImages[i]))[0] / N;
		}
	}
	diffMat.row(N) = VectorXd::Constant(cols, std::numeric_limits<double>::max());

	VectorXd patch_mean = VectorXd::Zero(cols),
		patch_stddev = VectorXd::Zero(cols);

	// Enhance local contrast
	MatrixXd diffEnhanced = MatrixXd::Zero(diffMat.rows(), diffMat.cols());
	for (int i=0; i<diffEnhanced.rows(); i++) {
		int lowerBound = std::max(0, i-localRadius/2);
		int size = std::min(i+localRadius/2, (int)diffEnhanced.size()-i);
		auto localPatch = diffMat.block(lowerBound,0,size,cols);

		for (int j=0; j<cols; j++) {
			meanStdDev(localPatch.col(j), patch_mean[j], patch_stddev[j], true);
		}
		diffEnhanced.row(i) = (diffMat.row(i) - patch_mean).cwiseProduct(patch_stddev.cwiseInverse());
	}

	double minVal = diffEnhanced.minCoeff();
	diffEnhanced = diffEnhanced - MatrixXd::Constant(diffEnhanced.rows(), diffEnhanced.cols(), minVal);

	return diffEnhanced;

}

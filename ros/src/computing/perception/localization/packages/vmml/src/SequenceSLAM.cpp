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
SequenceSLAM::learn (const cv::Mat &imgsrc, dataItemId frameId)
{
	cv::Mat preprocessedImg = normalizePatch(imgsrc, patchSize);
	learntNormalizedImages.push_back(preprocessedImg);
	int frameLrn = static_cast<int>(learntNormalizedImages.size()-1);
	dataSetMap.insert(make_pair(frameLrn, frameId));
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


//void
//SequenceSLAM::find(const cv::Mat &frame)
//{
//	Eigen::VectorXd diffVec = calculateDifferenceEnhancedVector(frame);
//
//	// findMatches
//
//	Eigen::VectorXd matches =
//		Eigen::VectorXd::Constant(matchingDistance/2,
//			std::numeric_limits<Eigen::VectorXd::Scalar>::max());
//
//}


void
SequenceSLAM::find (const cv::Mat &f, const int matchingDistance)
const
{
	vector<cv::Mat> MS = {f};
	return find(MS, matchingDistance);
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

	MatrixXd diffEnhancedMat = calculateDifferenceEnhancedMatrix(preprocessSrc);

	int mDist = matching_distance + (matching_distance % 2);
	int half_mDist = mDist / 2;

	MatrixXf matches =
		MatrixXf::Constant(2, diffEnhancedMat.cols(),
			std::numeric_limits<MatrixXf::Scalar>::max());

	// XXX: This code is still mysterious
	for (int N=half_mDist+1; N<(diffEnhancedMat.cols() - half_mDist); N++) {
		pair<int,double> match = findMatch(diffEnhancedMat, N, mDist);
		matches(0, N) = match.first;
		matches(1, N) = match.second;
	}

	// XXX: What are the returns ?
	return;
}


std::pair<int,double>
SequenceSLAM::findMatch
(const Eigen::MatrixXd &diffMat, const int N, const int m_dist)
const
{
	const int
		move_min = static_cast<int>(minVelocity * m_dist),
		move_max = static_cast<int>(maxVelocity * m_dist);

	/* Matching is based on max and min velocity */
	VectorXd velocity(move_max-move_min+1);
	for (int i=0; i<velocity.size(); i++)
		velocity[i] = double(move_min + i) / matchingDistance;

    /* Create incremental indices based on the previously calculated velocity */
	ArrayXXi increment_indices(move_max-move_min+1, matchingDistance+1);
	for (int y=0; y<increment_indices.rows(); y++) {
		double v_val = velocity(y, 0);
		for (int x=0; x<increment_indices.cols(); x++) {
			increment_indices(y, x) = static_cast<int>(floor(x*v_val));
		}
	}

	int y_max = diffMat.rows();

    /* Start trajectory */
	int n_start = N - (matchingDistance / 2);
	ArrayXXi X (velocity.rows(), matchingDistance+1);
	for (int i=0; i<X.cols(); i++)
		X.col(i) = ArrayXi::Constant(X.rows(), (n_start+i-1) * y_max);

	VectorXf score(diffMat.rows());

	for (int s=0; s<diffMat.rows(); s++) {

		ArrayXXi Y = increment_indices + s;
		Y = (Y>y_max).select(Y, ArrayXXi::Constant(Y.rows(), Y.cols(), y_max));
		ArrayXXi idx_mat = X + Y;

		float min_sum = std::numeric_limits<float>::max();
		for (int r=0; r<idx_mat.rows(); r++) {
			float sum = 0;

			for (int c=0; c<idx_mat.cols(); c++) {
				int idx = idx_mat(r, c);
				sum += diffMat(idx%y_max, idx%y_max);
			}
			min_sum = std::min(min_sum, sum);
		}

		score[s] = min_sum;
	}

    /* Find the lowest score */
	VectorXf::Index min_index_t;
	float min_score = score.minCoeff(&min_index_t);
	int min_index = static_cast<int>(min_index_t);

    /* ... now discard the RWindow region from where we found the lowest score ... */
	for (int i=max(0, min_index-RWindow/2);
		i<min(int(score.size()), min_index+RWindow/2);
		i++)
		score[i] = std::numeric_limits<float>::max();

    /* ... in order to find the second lowest score */
	decltype(score)::Index min_index_t2;
	float min_score_2 = score.minCoeff(&min_index_t2);
	int min_index_2 = static_cast<int>(min_index_t2);

	return make_pair(min_index_t+matchingDistance/2, min_score/min_score_2);
}


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


MatrixXd
SequenceSLAM::calculateDifferenceEnhancedMatrix (const std::vector<cv::Mat> &preprocSrc)
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

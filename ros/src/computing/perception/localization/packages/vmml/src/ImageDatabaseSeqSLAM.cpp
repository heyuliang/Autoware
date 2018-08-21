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
			float stddev_val = sqrtf((pow(patch_stddev.at<float>(0,0), 2) * patch_area) / (patch_area-1));

		}
	}

	return result;
}

/*
 * Mapper.h
 *
 *  Created on: Jun 5, 2018
 *      Author: sujiwo
 */

#ifndef MAPBUILDER_H_
#define MAPBUILDER_H_

#include <string>
#include <vector>
#include <set>
#include <memory>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "KeyFrame.h"
#include "MapPoint.h"
#include "VMap.h"
#include "CustomDataset.h"
#include "utilities.h"



class Viewer;


typedef std::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pointCloudPtr;


class MapBuilder
{
public:

	MapBuilder(const std::string &datasetDir);
	virtual ~MapBuilder();

	VMap* getMap()
	{ return cMap; }

	bool run2 (int startKeyfr=0, int maxKeyframes=0);

	void dump (const std::string &filename);

//	pointCloudPtr dumpPointCloud ();
	bool runBADB = true;

private:
	CustomDataset dataset;
	CameraPinholeParams cparams;

	VMap *cMap;

	Viewer *viewer;

	cv::Mat mask;

	void buildKeyFrames(int startIn=0, int maxNumOfFrames=0);

	cv::Mat vocabulary;
	void trainVocabulary ();

	KeyFrame* createKeyFrame (CustomDataItem &di,
		kfid setKfId=std::numeric_limits<kfid>::max());
};

#endif /* MAPBUILDER_H_ */

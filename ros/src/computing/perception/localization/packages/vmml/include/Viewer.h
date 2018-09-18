/*
 * Viewer.h
 *
 *  Created on: Jul 10, 2018
 *      Author: sujiwo
 */

#ifndef VIEWER_H_
#define VIEWER_H_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <thread>
#include "VMap.h"
#include "datasets/GenericDataset.h"



class Viewer {
public:

//	Viewer (const GenericDataset &genset);
	Viewer (GenericDataset::ConstPtr getset);
	~Viewer ();

	// Frame and dataItem may be decoupled
	void update (int dataItemId, kfid frameId);

	void setMap (VMap *m);

private:
	GenericDataset::ConstPtr dataset;
	VMap *cMap;
};

#endif /* VIEWER_H_ */

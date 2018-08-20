/*
 * Mapper.cpp
 *
 *  Created on: Jun 5, 2018
 *      Author: sujiwo
 */

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <opencv2/highgui.hpp>
#include <pcl/io/pcd_io.h>

#include "KeyFrame.h"
#include "MapBuilder.h"
#include "Viewer.h"
#include "optimizer.h"
#include "ImageDatabase.h"


using namespace std;
using namespace Eigen;
using namespace std::chrono;


#define MIN_NEW_POINTS 20


typedef Matrix<double,3,4> CameraIntrinsicMatrix;

const Eigen::Vector3d origin(0,0,0);

static int onlyCamera;


MapBuilder::MapBuilder(const string &datasetDir) :
	cMap(NULL),
	dataset (datasetDir)

{
	mask = dataset.getMask();
	cparams = dataset.getCameraParameter();

	cMap = new VMap(mask, FeatureDetectorT::ORB, DescriptorMatcherT::BruteForce);
	onlyCamera = cMap->addCameraParameter(cparams);

	viewer = new Viewer(dataset);
	viewer->setMap(cMap);

	cerr << "Expected #of frames: " << dataset.size() << endl;
}


MapBuilder::~MapBuilder()
{
	delete viewer;
}


void MapBuilder::buildKeyFrames (int startInN, int maxNumOfFrames)
{
	if (maxNumOfFrames==0 or startInN + maxNumOfFrames > dataset.size())
		maxNumOfFrames = dataset.size();

// XXX: KeyFrame initializations may not be parallelized as keyframeInvIdx.insert() operation
// is not thread-safe

//#pragma omp parallel
	for (uint i=startInN, p=0; i<startInN + maxNumOfFrames; i++, p++) {
		cerr << "Initialize " << p << '/' << maxNumOfFrames << endl;
		createKeyFrame (const_cast<CustomDataItem&>(dataset.at(i)), i);
	}
}


KeyFrame* MapBuilder::createKeyFrame (CustomDataItem &di, kfid setKfId)
{
	KeyFrame *mNewFrame;
	kfid kfid;

	cv::Mat itemImage;
	itemImage = di.getImage();

	kfid = cMap->createKeyFrame(
		itemImage,
		di.getPosition(),
		di.getOrientation(),
		onlyCamera,
		&mNewFrame,
		setKfId);
	return mNewFrame;
}


bool MapBuilder::run2 (int startKeyfr, int maxNumOfKeyframes)
{
	if (maxNumOfKeyframes==0)
		maxNumOfKeyframes = dataset.size();
	cout << "Initializing...\n";
	buildKeyFrames(startKeyfr, maxNumOfKeyframes);
	vector<kfid> kfList = cMap->getKeyFrameList();

	// Initialize map
	viewer->update(kfList[0], kfList[0]);
	cMap->estimateStructure(kfList[0], kfList[1]);
	cout << "Map initialized\n";
	viewer->update(kfList[1], kfList[1]);

	for (int i=2; i<kfList.size(); i++) {
		kfid fromKfId = kfList[i-1],
			toKfId = kfList[i];
		cMap->estimateAndTrack(fromKfId, toKfId);
		viewer->update(toKfId, toKfId);
		cout << i << '/' << kfList.size() << endl;
	}

	system_clock::time_point t1 = system_clock::now();

	if (runBADB) {

//		thread ba([this] {
					cout << "Bundling...";
					bundle_adjustment(cMap);
					cout << "Done\n";
//		});

//		thread db([this] {
					cout << "Rebuilding Image DB... ";
					cout.flush();
					cMap->getImageDB()->rebuildAll();
					cout << "Done\n";
//		});

//		ba.join();
//		db.join();
	}

	system_clock::time_point t2 = system_clock::now();
	duration<float> td = t2 - t1;
	cerr << "Time(s): " << td.count() << endl;

	return true;
}


void MapBuilder::dump (const std::string &filename)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr vizCloud =
		cMap->dumpPointCloudFromMapPoints();
	pcl::io::savePCDFileBinary(filename, *vizCloud);
}

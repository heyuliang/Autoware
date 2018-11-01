/*
 * MapBuilder2.cpp
 *
 *  Created on: Jul 26, 2018
 *      Author: sujiwo
 */

#include <exception>
#include <thread>
#include "MapBuilder2.h"
#include "Optimizer.h"
#include "ImageDatabase.h"
#include "Viewer.h"
#include "utilities.h"



using namespace std;
using namespace Eigen;


MapBuilder2::MapBuilder2() :
	kfAnchor(0)
{
	cMap = new VMap();
//	imageView = new Viewer;
}


MapBuilder2::~MapBuilder2()
{
//	delete(imageView);
}


void
MapBuilder2::initialize (const InputFrame &f1, const InputFrame &f2)
{
	kfid k1 = cMap->createKeyFrame(f1.image, f1.position, f1.orientation, f1.cameraId, NULL, f1.sourceId, f1.tm);
	kfid k2 = cMap->createKeyFrame(f2.image, f2.position, f2.orientation, f2.cameraId, NULL, f2.sourceId, f2.tm);
	cMap->estimateStructure(k1, k2);
	kfAnchor = k1;
	ifrAnchor = f1;
}


void
MapBuilder2::track (const InputFrame &f)
{
	if (initialized==false)
		throw runtime_error("Map not initialized");

	kfid fId = cMap->createKeyFrame(f.image, f.position, f.orientation, f.cameraId, NULL, f.sourceId, f.tm);
	cMap->estimateAndTrack(kfAnchor, fId);

	// XXX: Decide when to move the anchor
	kfAnchor = fId;
	ifrAnchor = f;
}


void
MapBuilder2::input(const InputFrame &f)
{
	if (isNormalFrame(f)==false)
		return;

	double runTrans, runRot;

	if (initialized==false) {

		if (frame0.image.empty()) {
			frame0 = f;
			return;
		}

		else {
			f.getPose().displacement(frame0.getPose(), runTrans, runRot);
			if (runTrans>=translationThrs or runRot>=rotationThrs) {
				initialize(frame0, f);
				inputCallback(f);
				initialized = true;
				cerr << "Initialized; # of map points: " << cMap->allMapPoints().size() << endl;
				return;
			}
		}
	}

	else {
		ifrAnchor.getPose().displacement(f.getPose(), runTrans, runRot);
		if (runTrans>=translationThrs or runRot>=rotationThrs) {
			kfid lastAnchor = kfAnchor;
			track (f);

			// Build connections
			vector<kfid> kfInsToAnchor = cMap->getKeyFramesComeInto(lastAnchor);
			cerr << "Found " << kfInsToAnchor.size() << " input keyframes\n";
			const kfid targetKfId = kfAnchor;
			// XXX: Parallelize this
			for (auto &kfx: kfInsToAnchor) {
				cMap->trackMapPoints(kfx, targetKfId);
			}

			inputCallback(f);
		}
	}
}


/*
 * This function decides when a frame is `good enough' in terms of exposure
 * to be included for map building
 */
bool
MapBuilder2::isNormalFrame (const InputFrame &f)
{
	// throw away over-exposed frames
	// XXX: also need to do the same for under-exposed frame
	auto normcdf = cdf(f.image);
	if (normcdf[127] < 0.25)
		return false;
	else return true;
}


void
MapBuilder2::build ()
{
	mapPointCulling();
	cMap->fixFramePointsInv();

	thread ba([this] {
				cout << "Bundling...";
				bundle_adjustment(cMap);
				cout << "BA Done\n";
	});

	thread db([this] {
				cout << "Rebuilding Image DB... ";
				cout.flush();
				cMap->getImageDB()->rebuildAll();
				cout << "Image DB Build Done\n";
	});

	ba.join();
	db.join();
	return;
}


void
MapBuilder2::runFromDataset(GenericDataset *ds)
{
	sourceDataset = ds;
	if (initialized != false)
		throw runtime_error("Map process has been running; aborted");

	initialized = true;

	this->build();
}


/*
 * XXX: Unfinished
 */
void
MapBuilder2::mapPointCulling()
{
	cout << "Culling points...";

	vector<mpid> mpToRemove;
	const int minKfRelatedFromMP = 3;
	auto allMapPoints = cMap->allMapPoints();
	const int N = allMapPoints.size();

	for (auto &mp: allMapPoints) {

		auto relatedKfs = cMap->getRelatedKeyFrames(mp);
		if (relatedKfs.size() < minKfRelatedFromMP) {
			mpToRemove.push_back(mp);
		}
	}

	for (auto mp: mpToRemove) {
		cMap->removeMapPoint(mp);
	}

	cout << "Removed " << mpToRemove.size() << " out of " << N << " points";
}


void
MapBuilder2::setMask(const cv::Mat &m)
{
	mask = m.clone();
	cMap->setMask(mask);
}

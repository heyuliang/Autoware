/*
 * MapBuilder2.cpp
 *
 *  Created on: Jul 26, 2018
 *      Author: sujiwo
 */

#include <exception>
#include <thread>
#include "MapBuilder2.h"
#include "optimizer.h"
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
	kfid k1 = cMap->createKeyFrame(f1.image, f1.position, f1.orientation, f1.cameraId, NULL, f1.setKfId, f1.tm);
	kfid k2 = cMap->createKeyFrame(f2.image, f2.position, f2.orientation, f2.cameraId, NULL, f2.setKfId, f2.tm);
	cMap->estimateStructure(k1, k2);
	kfAnchor = k1;
	ifrAnchor = f1;
}


void
MapBuilder2::track (const InputFrame &f)
{
	if (initialized==false)
		throw runtime_error("Map not initialized");

	kfid fId = cMap->createKeyFrame(f.image, f.position, f.orientation, f.cameraId, NULL, f.setKfId, f.tm);
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
				return;
			}
		}
	}

	else {
		ifrAnchor.getPose().displacement(f.getPose(), runTrans, runRot);
		if (runTrans>=translationThrs or runRot>=rotationThrs) {
			kfid lastAnchor = kfAnchor;
			track (f);
			inputCallback(f);

			// Build connections
			vector<kfid> kfInsToAnchor = cMap->getKeyFramesComeInto(lastAnchor);
			const kfid targetKfId = kfAnchor;
			// XXX: Parallelize this
			for (auto &kfx: kfInsToAnchor) {
				cMap->trackMapPoints(kfx, targetKfId);
			}
		}
	}
}


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
	thread ba([this] {
				cout << "Bundling...";
				bundle_adjustment(cMap);
				cout << "Done\n";
	});

	thread db([this] {
				cout << "Rebuilding Image DB... ";
				cout.flush();
				cMap->getImageDB()->rebuildAll();
				cout << "Done\n";
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
		throw runtime_error("Map has been running; aborted");

	initialized = true;

	this->build();
}

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



using namespace std;
using namespace Eigen;


MapBuilder2::MapBuilder2() :
	currentAnchor(0)
{
	cMap = new VMap();
//	imageView = new Viewer;
}


MapBuilder2::~MapBuilder2()
{
}


void
MapBuilder2::initialize (const InputFrame &f1, const InputFrame &f2)
{
	kfid k1 = cMap->createKeyFrame(f1.image, f1.position, f1.orientation, f1.cameraId);
	kfid k2 = cMap->createKeyFrame(f2.image, f2.position, f2.orientation, f2.cameraId);
	cMap->estimateStructure(k1, k2);
	currentAnchor = k2;
	initialized = true;
}


void
MapBuilder2::track (const InputFrame &f)
{
	if (initialized==false)
		throw runtime_error("Map not initialized");

	kfid fId = cMap->createKeyFrame(f.image, f.position, f.orientation, f.cameraId);
	cMap->estimateAndTrack(currentAnchor, fId);
	currentAnchor = fId;
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

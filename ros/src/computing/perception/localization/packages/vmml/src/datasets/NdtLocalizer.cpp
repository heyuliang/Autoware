/*
 * NdtLocalizer.cpp
 *
 *  Created on: Sep 21, 2018
 *      Author: sujiwo
 */

#include <stdlib.h>
#include <pcl/io/pcd_io.h>

#include "src/datasets/NdtLocalizer.h"

using namespace std;
using pcl::PointXYZ;
using namespace Eigen;


inline double nrand(double n)
{
	double r;
	r = n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX);
	return r;
}


NdtLocalizer::NdtLocalizer(const NdtLocalizerInitialConfig &initialConfig) :

	ndMap(initialize_NDmap())

{}


void
NdtLocalizer::loadMap (const std::string &filename)
{
	pcl::PointCloud<PointXYZ>::Ptr mapInp (new pcl::PointCloud<PointXYZ>);

	if (mapLoaded==false) {

		pcl::PCDReader fReader;
		fReader.read(filename, *mapInp);
		loadMap(mapInp);
	}
}


NdtLocalizer::~NdtLocalizer()
{}


void
NdtLocalizer::putEstimation (const Pose &pEst)
{
	Vector3d rotations = quaternionToRPY(pEst.orientation());
	prev_pose.x, prev_pose.y, prev_pose.z =
		pEst.position().x(), pEst.position().y(), pEst.position().z();
	prev_pose.theta, prev_pose.theta2, prev_pose.theta3 =
		rotations.x(), rotations.y(), rotations.z();
	prev_pose2 = prev_pose;
}


void
NdtLocalizer::loadMap (pcl::PointCloud<PointXYZ>::ConstPtr mapcloud)
{
	for (auto pointIt=mapcloud->begin(); pointIt!=mapcloud->end(); ++pointIt) {
		auto &pointSrc = *pointIt;
		Point pnd = {
			pointSrc.x,
			pointSrc.y,
			pointSrc.z};
		add_point_map(ndMap, &pnd);
	}

	mapLoaded = true;
}


Pose
NdtLocalizer::localize (pcl::PointCloud<pcl::PointXYZ>::ConstPtr scan)
{
	vector<Point> ndtScanPoints (scan->size());
	for (int i=0, j=0; i<scan->size(); i++) {
		auto &p = scan->at(i);
		auto &pt = ndtScanPoints.at(i);
		pt.x = p.x + nrand(0.01);
		pt.y = p.y + nrand(0.01);
		pt.z = p.z + nrand(0.01);
		double dist = pt.x*pt.x + pt.y*pt.y + pt.z*pt.z;
		if (dist < 3*3) {

		}
		j++;
	}
}

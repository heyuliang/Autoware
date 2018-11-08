/*
 * VectorMapDisplay.cpp
 *
 *  Created on: Nov 8, 2018
 *      Author: sujiwo
 */

#include "VectorMapDisplay.h"


void insertMarkerArray(visualization_msgs::MarkerArray& a1, const visualization_msgs::MarkerArray& a2)
{
	a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
}


VectorMapLoader::VectorMapLoader(const std::string &directory) :
	vmDir(directory),
	vector_map::VectorMap()
{
	loadAll ();
}


void VectorMapLoader::loadAll()
{

}


VectorMapDisplay::VectorMapDisplay()
{
	// TODO Auto-generated constructor stub

}

VectorMapDisplay::~VectorMapDisplay()
{
	// TODO Auto-generated destructor stub
}


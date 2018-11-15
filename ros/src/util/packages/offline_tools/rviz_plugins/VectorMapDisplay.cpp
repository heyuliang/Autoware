/*
 * VectorMapDisplay.cpp
 *
 *  Created on: Nov 8, 2018
 *      Author: sujiwo
 */

#include "VectorMapDisplay.h"
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/int_property.h>


using namespace std;


VectorMapDisplay::VectorMapDisplay():
	rviz::MarkerArrayDisplay()
{
	vMapDir_ = new rviz::StringProperty(
		"Vector Map Directory",
		QString(),
		"Directory of Autoware Vector Map Files",
		this,
		SLOT( changeDir() ));

	marker_topic_property_->hide();
	queue_size_property_->hide();
}

VectorMapDisplay::~VectorMapDisplay()
{
}


void
VectorMapDisplay::onInitialize()
{

}


void
VectorMapDisplay::changeDir()
{
	const string vectorMapDirName = vMapDir_->getStdString();
	mapData = shared_ptr<VectorMapLoader> (new VectorMapLoader(vectorMapDirName));

	markers_.clear();

	for (auto &markerM: mapData->marker_array.markers) {
		rviz::MarkerBasePtr marker(rviz::createMarker(markerM.type, this, context_, scene_node_));
		markers_.insert(marker);
	}

	context_->queueRender();
}


void
VectorMapDisplay::onEnable()
{
}


void
VectorMapDisplay::onDisable()
{

}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(VectorMapDisplay, rviz::Display)

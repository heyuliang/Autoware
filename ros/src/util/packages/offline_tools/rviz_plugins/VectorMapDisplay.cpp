/*
 * VectorMapDisplay.cpp
 *
 *  Created on: Nov 8, 2018
 *      Author: sujiwo
 */

#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/int_property.h>

#include "VectorMapDisplay.h"


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
//	namespaces_category_->hide();
}

VectorMapDisplay::~VectorMapDisplay()
{
	// TODO Auto-generated destructor stub
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

	incomingMarkerArray(mapData->ConstPtr());
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

/*
 * PCDMap.cpp
 *
 *  Created on: Sep 30, 2018
 *      Author: sujiwo
 */

#include "PCDFileDisplay.h"


PCDFileDisplay::PCDFileDisplay() :
	rviz::PointCloudDisplay()
{
	pcdfile_ = new rviz::StringProperty("PCDFilePath", "", "PCD File to load");
}


PCDFileDisplay::~PCDFileDisplay()
{
}


void
PCDFileDisplay::onInitialize()
{
	point_cloud_common_->initialize(context_, scene_node_);
}


void
PCDFileDisplay::reset()
{}


//void
//PCDFileDisplay::update( float wall_dt, float ros_dt )
//{}





#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PCDFileDisplay, rviz::Display)

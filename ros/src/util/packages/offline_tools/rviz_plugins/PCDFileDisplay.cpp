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


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PCDFileDisplay, rviz::Display)

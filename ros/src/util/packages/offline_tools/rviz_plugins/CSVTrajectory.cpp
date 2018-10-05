/*
 * CSVTrajectory.cpp
 *
 *  Created on: Oct 4, 2018
 *      Author: sujiwo
 */

#include "CSVTrajectory.h"

CSVTrajectory::CSVTrajectory()
{
	csvFilePath = new FileProperty(
			"CSVFilePath",
			QString(),
			"CSV File to load",
			this,
			SLOT( changeFile() ));

	topic_property_->hide();

	rviz::PathDisplay();
}

CSVTrajectory::~CSVTrajectory()
{
}


void CSVTrajectory::changeFile()
{

}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(CSVTrajectory, rviz::Display)

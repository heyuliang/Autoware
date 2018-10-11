/*
 * CSVTrajectory.cpp
 *
 *  Created on: Oct 4, 2018
 *      Author: sujiwo
 */

#include <cstdlib>
#include <nav_msgs/Path.h>
#include <OgreMatrix4.h>
#include "CSVTrajectory.h"
#include "csv.h"

using namespace std;


CSVTrajectory::CSVTrajectory()
{
	csvFilePath = new rviz::StringProperty(
			"CSVFilePath",
			QString(),
			"CSV File to load",
			this,
			SLOT( changeFile() ));

	axesLengthSize = new rviz::FloatProperty( "Length", 0.3,
						"Length of the axes.",
						this, SLOT(updatePoseAxisGeometry()) );
	axesRadius = new rviz::FloatProperty( "Radius", 0.03,
						"Radius of the axes.",
						this, SLOT(updatePoseAxisGeometry()) );
}

CSVTrajectory::~CSVTrajectory()
{
}


void CSVTrajectory::onInitialize()
{
}


void CSVTrajectory::changeFile()
{
	string fn = csvFilePath->getString().toStdString();
	return updateDisplay(fn);
}


void
CSVTrajectory::updatePoseAxisGeometry()
{
	for (int i=0; i<axesList.size(); i++) {
		axesList[i]->set(axesLengthSize->getFloat(), axesRadius->getFloat());
	}

	// last step
	queueRender();
}


void
CSVTrajectory::updateDisplay (const std::string &filename)
{
	try {
		StringTable csvTableInp = create_table(filename);
		if (csvTableInp.columns() != 8)
			throw runtime_error("Input file must contains 8 columns without header");
		axesList.resize(csvTableInp.rows());

		for (int r=0; r<csvTableInp.rows(); ++r) {

			axesList[r] = new rviz::Axes(
				scene_manager_,
				scene_node_,
				axesLengthSize->getFloat(),		// Axes length; must be set using property
				axesRadius->getFloat());		// Axes radius

			axesList[r]->setPosition(Ogre::Vector3 (
					csvTableInp.getd(r, 1),
					csvTableInp.getd(r, 2),
					csvTableInp.getd(r, 3)));

			axesList[r]->setOrientation(Ogre::Quaternion(
				csvTableInp.getd(r, 7),
				csvTableInp.getd(r, 4),
				csvTableInp.getd(r, 5),
				csvTableInp.getd(r, 6)));
		}
	} catch (std::runtime_error &e) {
		// Catch errors here
		// XXX: Put nice error message when unable to open input file
	}

	queueRender();
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(CSVTrajectory, rviz::Display)

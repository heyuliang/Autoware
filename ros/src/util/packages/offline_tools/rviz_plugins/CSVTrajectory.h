/*
 * CSVTrajectory.h
 *
 *  Created on: Oct 4, 2018
 *      Author: sujiwo
 */

#ifndef _CSVTRAJECTORY_H_
#define _CSVTRAJECTORY_H_

#include <string>
#include <vector>
#include <rviz/properties/string_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/display.h>
#include "FileProperty.h"


class CSVTrajectory : public rviz::Display
{
Q_OBJECT
public:
	CSVTrajectory();
	virtual ~CSVTrajectory();


protected:
//	void onInitialize();

private
	Q_SLOTS:
	void changeFile();
	void updatePoseAxisGeometry();

protected:
	virtual void onInitialize();

//	FileProperty* csvFilePath;
	rviz::StringProperty* csvFilePath;
	std::vector<rviz::Axes*> axesList;

	rviz::FloatProperty* axesLengthSize;
	rviz::FloatProperty* axesRadius;

private:
	void updateDisplay (const std::string &filename);
//	void updateDisplay ()
};

#endif /* _CSVTRAJECTORY_H_ */

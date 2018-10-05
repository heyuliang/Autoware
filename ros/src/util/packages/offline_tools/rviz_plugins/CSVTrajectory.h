/*
 * CSVTrajectory.h
 *
 *  Created on: Oct 4, 2018
 *      Author: sujiwo
 */

#ifndef _CSVTRAJECTORY_H_
#define _CSVTRAJECTORY_H_


#include <rviz/properties/string_property.h>
#include <rviz/default_plugin/path_display.h>
#include <rviz/display.h>

#include "FileProperty.h"


class CSVTrajectory : public rviz::PathDisplay
{
Q_OBJECT
public:
	CSVTrajectory();
	virtual ~CSVTrajectory();


private
	Q_SLOTS:
	void changeFile();

protected:
//	virtual void onInitialize();

	FileProperty* csvFilePath;
};

#endif /* _CSVTRAJECTORY_H_ */

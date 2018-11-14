/*
 * VectorMapDisplay.h
 *
 *  Created on: Nov 8, 2018
 *      Author: sujiwo
 */

#ifndef _VECTORMAPDISPLAY_H_
#define _VECTORMAPDISPLAY_H_

#include <rviz/display.h>
#include <rviz/default_plugin/marker_array_display.h>
#include "VectorMapLoader.h"


class VectorMapDisplay: public rviz::MarkerArrayDisplay
{
Q_OBJECT

public:
	VectorMapDisplay();
	virtual ~VectorMapDisplay();

private
	Q_SLOTS:

protected:
	virtual void onInitialize();
};

#endif /* _VECTORMAPDISPLAY_H_ */

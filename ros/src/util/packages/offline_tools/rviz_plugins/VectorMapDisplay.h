/*
 * VectorMapDisplay.h
 *
 *  Created on: Nov 8, 2018
 *      Author: sujiwo
 */

#ifndef _VECTORMAPDISPLAY_H_
#define _VECTORMAPDISPLAY_H_

#include <memory>

#include <rviz/display.h>
#include <rviz/default_plugin/marker_array_display.h>
#include <rviz/properties/string_property.h>
#include "VectorMapLoader.h"


// XXX: Please change to rviz::Display to bypass
class VectorMapDisplay: public rviz::MarkerArrayDisplay
{
Q_OBJECT

public:
	VectorMapDisplay();
	virtual ~VectorMapDisplay();

public
	Q_SLOTS:

private
	Q_SLOTS:
	void changeDir ();

protected:
	virtual void onInitialize();

	rviz::StringProperty *vMapDir_;

	std::shared_ptr<VectorMapLoader> mapData = nullptr;

	virtual void onEnable();
	virtual void onDisable();
};

#endif /* _VECTORMAPDISPLAY_H_ */

/*
 * VectorMapDisplay.h
 *
 *  Created on: Nov 8, 2018
 *      Author: sujiwo
 */

#ifndef _VECTORMAPDISPLAY_H_
#define _VECTORMAPDISPLAY_H_

#include <memory>
#include <set>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <rviz/default_plugin/marker_array_display.h>
#include <rviz/properties/string_property.h>
#include <rviz/default_plugin/markers/marker_base.h>
#include <rviz/default_plugin/marker_utils.h>
#include "rviz/default_plugin/markers/marker_base.h"
#include "rviz/default_plugin/marker_utils.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/ogre_helpers/billboard_line.h"
#include "rviz/ogre_helpers/shape.h"

#include "VectorMapLoader.h"


//typedef std::shared_ptr<rviz::MarkerBase> MarkerBasePtr;


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

	std::shared_ptr<VectorMapLoader> mapData;

	std::set<rviz::MarkerBasePtr> markers_;

	virtual void onEnable();
	virtual void onDisable();
};

#endif /* _VECTORMAPDISPLAY_H_ */

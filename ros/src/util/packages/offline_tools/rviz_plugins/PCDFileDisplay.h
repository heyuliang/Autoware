/*
 * PCDMap.h
 *
 *  Created on: Sep 30, 2018
 *      Author: sujiwo
 */

#ifndef _RVIZ_PLUGINS_PCDMAP_H_
#define _RVIZ_PLUGINS_PCDMAP_H_


#include <rviz/default_plugin/point_cloud_display.h>
#include <rviz/default_plugin/point_cloud_common.h>
#include <rviz/properties/string_property.h>
#include <rviz/display.h>


class PCDFileDisplay : public rviz::PointCloudDisplay
{
Q_OBJECT
public:
	PCDFileDisplay();
	virtual ~PCDFileDisplay();

	void update( float wall_dt, float ros_dt )
	{}

	void reset();

protected:
	virtual void onInitialize();


private Q_SLOTS:


private:
	rviz::StringProperty* pcdfile_;
};

#endif /* _RVIZ_PLUGINS_PCDMAP_H_ */

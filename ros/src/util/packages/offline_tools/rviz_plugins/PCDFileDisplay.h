/*
 * PCDMap.h
 *
 *  Created on: Sep 30, 2018
 *      Author: sujiwo
 */

#ifndef _RVIZ_PLUGINS_PCDMAP_H_
#define _RVIZ_PLUGINS_PCDMAP_H_

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <rviz/properties/enum_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/display.h>


class PCDFileDisplay : public rviz::Display
{
Q_OBJECT
public:
	PCDFileDisplay();
	virtual ~PCDFileDisplay();

//	void update( float wall_dt, float ros_dt )
//	{}
//
//	void reset();

protected:
	virtual void onInitialize();


private
	Q_SLOTS:
	void changeFile();
	void updateStyle();
	void updateBillboardSize();


private:
	rviz::StringProperty* pcdfile_;
	rviz::EnumProperty* style_property_;
	rviz::FloatProperty* point_world_size_property_;
	rviz::FloatProperty* point_pixel_size_property_;

	boost::shared_ptr<rviz::PointCloud> cloud_render_;
	std::vector<rviz::PointCloud::Point> pointList;

private:
	void updateDisplay(const std::string &loadThisFile);

};

#endif /* _RVIZ_PLUGINS_PCDMAP_H_ */

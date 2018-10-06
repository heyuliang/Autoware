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
#include <sensor_msgs/PointCloud2.h>

#include <rviz/properties/enum_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/display.h>


class PCDFileDisplay : public rviz::Display
{
Q_OBJECT
public:
	PCDFileDisplay();
	virtual ~PCDFileDisplay();

	enum {
		FLAT_COLOR,
		Z_COLOR
	};

protected:
	virtual void onInitialize();

public
	Q_SLOTS:
	void causeRetransform();


private
	Q_SLOTS:
	void changeFile();
	void updateStyle();
	void updateBillboardSize();
	void updateColorTransformer();


private:
	rviz::StringProperty* pcdfile_;
	rviz::EnumProperty* style_property_;
	rviz::FloatProperty* point_world_size_property_;
	rviz::FloatProperty* point_pixel_size_property_;

	sensor_msgs::PointCloud2::Ptr cloudMsg_;
	boost::shared_ptr<rviz::PointCloud> cloud_render_;
	std::vector<rviz::PointCloud::Point> pointList;

	rviz::EnumProperty* colorChooser_;

	rviz::AxisColorPCTransformer* axesColorTransform_;
	QList<rviz::Property*> axesColorTransformProps;

	rviz::FlatColorPCTransformer* flatColorTransform_;
	QList<rviz::Property*> flatColorTransformProps;

	rviz::PointCloudTransformer* activeTransform_ = NULL;

private:
	void updatePointCloud(const std::string &loadThisFile);

	void updateDisplay();
};

#endif /* _RVIZ_PLUGINS_PCDMAP_H_ */

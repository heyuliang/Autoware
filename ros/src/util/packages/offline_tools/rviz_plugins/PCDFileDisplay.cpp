/*
 * PCDMap.cpp
 *
 *  Created on: Sep 30, 2018
 *      Author: sujiwo
 */

#include <OGRE/OgreSceneManager.h>
#include <rviz/properties/status_property.h>
#include "PCDFileDisplay.h"


using namespace std;
//using pcl::PointCloud;
//using pcl::PointXYZ;


PCDFileDisplay::PCDFileDisplay() :
	rviz::Display()
{
	pcdfile_ = new rviz::StringProperty(
		"PCDFilePath",
		QString(),
		"PCD File to load",
		this,
		SLOT( changeFile() ));

	cloud_render_ = boost::shared_ptr<rviz::PointCloud>(new rviz::PointCloud());

	style_property_ = new rviz::EnumProperty( "Style", "Flat Squares",
									  "Rendering mode to use, in order of computational complexity.",
									  this, SLOT( updateStyle() ), this );
	style_property_->addOption( "Points", rviz::PointCloud::RM_POINTS );
	style_property_->addOption( "Squares", rviz::PointCloud::RM_SQUARES );
	style_property_->addOption( "Flat Squares", rviz::PointCloud::RM_FLAT_SQUARES );
	style_property_->addOption( "Spheres", rviz::PointCloud::RM_SPHERES );
	style_property_->addOption( "Boxes", rviz::PointCloud::RM_BOXES );

	point_world_size_property_ = new rviz::FloatProperty( "Size (m)", 0.01,
												"Point size in meters.",
												this, SLOT( updateBillboardSize() ), this );
	point_world_size_property_->setMin( 0.0001 );

	point_pixel_size_property_ = new rviz::FloatProperty( "Size (Pixels)", 3,
												"Point size in pixels.",
												this, SLOT( updateBillboardSize() ), this );
	point_pixel_size_property_->setMin( 1 );

}


PCDFileDisplay::~PCDFileDisplay()
{
}


void
PCDFileDisplay::onInitialize()
{}


void
PCDFileDisplay::changeFile()
{
	const string filename = pcdfile_->getString().toStdString();
	return updateDisplay(filename);
}


void
PCDFileDisplay::updateStyle()
{
	rviz::PointCloud::RenderMode mode = (rviz::PointCloud::RenderMode) style_property_->getOptionInt();
	cloud_render_->setRenderMode(mode);
}


void
PCDFileDisplay::updateDisplay(const std::string &loadThisFile)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_open_ = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_render_->clear();

	pcl::PCDReader fileReader;
	try {
		fileReader.read(loadThisFile, *cloud_open_);

		// Do something with this pointcloud
		rviz::PointCloud::RenderMode renderMode = (rviz::PointCloud::RenderMode) style_property_->getOptionInt();

		// XXX: Add color transformer

		pointList.clear();
		pointList.resize(cloud_open_->width * cloud_open_->height);
		int i = 0;
		for (auto it=cloud_open_->begin(); it!=cloud_open_->end(); ++it) {
			pcl::PointXYZ &p = *it;
			rviz::PointCloud::Point pn;
			pn.position = Ogre::Vector3(p.x, p.y, p.z);
			pointList.at(i) = pn;
			++i;
		}

		cloud_render_->setRenderMode(renderMode);
		cloud_render_->addPoints(pointList.data(), pointList.size());
		scene_node_->attachObject(cloud_render_.get());

	} catch (exception &e) {
		// put error in rviz status
	}
}


void PCDFileDisplay::updateBillboardSize ()
{
	rviz::PointCloud::RenderMode mode = (rviz::PointCloud::RenderMode) style_property_->getOptionInt();
	float size;
	if( mode == rviz::PointCloud::RM_POINTS ) {
		size = point_pixel_size_property_->getFloat();
	} else {
		size = point_world_size_property_->getFloat();
	}
	cloud_render_->setDimensions(size, size, size);

//	context_->queueRender();
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PCDFileDisplay, rviz::Display)

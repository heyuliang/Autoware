/*
 *  Copyright (c) 2018, TierIV, Inc
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PolygonStamped.h>
#include <autoware_config_msgs/ConfigFootprintFilter.h>

class FootprintFilter
{

public:
	FootprintFilter();

private:
	void configCallback(const autoware_config_msgs::ConfigFootprintFilter::ConstPtr& config_msg_ptr);
	void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& sensorTF_in_cloud_msg_ptr);

	void publishFootPrintPolygon(const double front, const double rear, const double left, const double right, const double top, const double bottom, const ros::Time ros_time);

	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	tf::TransformListener tf_listener_;

	ros::Subscriber config_sub_;
	ros::Subscriber cloud_sub_;
	ros::Publisher 	cloud_pub_;
	ros::Publisher 	body_polygon_pub_;

	std::string baselink_frame_;
	bool use_vehicle_info_param_;
	double center_to_base_;
	double length_;
	double width_;
	double height_;
};

FootprintFilter::FootprintFilter()
	:nh_()
	,nh_private_("~")
	,baselink_frame_("/base_link")
	,use_vehicle_info_param_(true)
	,center_to_base_(1.0)
	,length_(5.0)
	,width_(2.0)
	,height_(2.5)
{
	nh_private_.param<std::string>("baselink_frame",  baselink_frame_, baselink_frame_);
	nh_private_.param("use_vehicle_info_param",  use_vehicle_info_param_, use_vehicle_info_param_);
	if(use_vehicle_info_param_) {
		nh_.param("/actuation/vehicle_info/center_to_base",  center_to_base_, center_to_base_);
		nh_.param("/actuation/vehicle_info/length", length_, length_);
		nh_.param("/actuation/vehicle_info/width", width_, width_);
		nh_.param("/actuation/vehicle_info/height", height_, height_);
	}
	else {
		nh_private_.param("center_to_base", center_to_base_, center_to_base_);
		nh_private_.param("length", length_, length_);
		nh_private_.param("width", width_, width_);
		nh_private_.param("height", height_, height_);
	}

	config_sub_ = nh_.subscribe("/config/footprint_filter", 10, &FootprintFilter::configCallback, this);
	cloud_sub_ = nh_.subscribe("/points_raw", 10, &FootprintFilter::pointCloudCallback, this);
	cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_clipped", 10);
	body_polygon_pub_ = nh_private_.advertise<geometry_msgs::PolygonStamped>("body_polygon", 10);
}

void FootprintFilter::configCallback(const autoware_config_msgs::ConfigFootprintFilter::ConstPtr& config_msg_ptr)
{
  use_vehicle_info_param_ = config_msg_ptr->use_vehicle_info_param;
  if(use_vehicle_info_param_) {
	  nh_.param("/actuation/vehicle_info/center_to_base",  center_to_base_, center_to_base_);
	  nh_.param("/actuation/vehicle_info/length", length_, length_);
	  nh_.param("/actuation/vehicle_info/width", width_, width_);
	  nh_.param("/actuation/vehicle_info/height", height_, height_);
  }
  else {
	  center_to_base_ = config_msg_ptr->center_to_base;
	  length_ = config_msg_ptr->length;
	  width_ = config_msg_ptr->width;
	  height_ = config_msg_ptr->height;

  }
}

void FootprintFilter::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& sensorTF_in_cloud_msg_ptr)
{
	std::string sensor_frame = sensorTF_in_cloud_msg_ptr->header.frame_id;
	ros::Time sensor_time = sensorTF_in_cloud_msg_ptr->header.stamp;

	sensor_msgs::PointCloud2::Ptr baselinkTF_in_cloud_msg_ptr(new sensor_msgs::PointCloud2);
    try {
      tf_listener_.waitForTransform(baselink_frame_, sensor_frame, sensor_time, ros::Duration(1.0));
      pcl_ros::transformPointCloud(baselink_frame_, *sensorTF_in_cloud_msg_ptr, *baselinkTF_in_cloud_msg_ptr, tf_listener_);
    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("Transform error: %s", ex.what());
      return;
    }

	pcl::PointCloud<pcl::PointXYZI>::Ptr baselinkTF_in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*baselinkTF_in_cloud_msg_ptr, *baselinkTF_in_cloud_ptr);

	pcl::PointCloud<pcl::PointXYZI>::Ptr baselinkTF_filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	baselinkTF_filtered_cloud_ptr->points.reserve( baselinkTF_in_cloud_ptr->points.size() );

	const double front  =  length_ / 2.0 + center_to_base_;
	const double rear   = -length_ / 2.0 + center_to_base_;
	const double left   =  width_ / 2.0;
	const double right  = -width_ / 2.0;
	const double top    =  height_;
	const double bottom = 0;

	for (const auto& point : baselinkTF_in_cloud_ptr->points) {
		if (point.x > front || point.x < rear  ||
		    point.y > left  || point.y < right ||
			point.z > top   || point.z < bottom  )
		{
			baselinkTF_filtered_cloud_ptr->points.push_back(point);
		}
	}

	sensor_msgs::PointCloud2::Ptr baselinkTF_filtered_cloud_msg_ptr(new sensor_msgs::PointCloud2);
	pcl::toROSMsg(*baselinkTF_filtered_cloud_ptr, *baselinkTF_filtered_cloud_msg_ptr);
	baselinkTF_filtered_cloud_msg_ptr->header.frame_id = baselink_frame_;
	baselinkTF_filtered_cloud_msg_ptr->header.stamp = sensor_time;

	sensor_msgs::PointCloud2::Ptr sensorTF_filtered_cloud_msg_ptr(new sensor_msgs::PointCloud2);
	try {
	  pcl_ros::transformPointCloud(sensor_frame, *baselinkTF_filtered_cloud_msg_ptr, *sensorTF_filtered_cloud_msg_ptr, tf_listener_);
    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("Transform error: %s", ex.what());
      return;
    }

	cloud_pub_.publish(sensorTF_filtered_cloud_msg_ptr);
	publishFootPrintPolygon(front, rear, left, right, top, bottom, sensor_time);
}

void FootprintFilter::publishFootPrintPolygon(const double front, const double rear, const double left, const double right, const double top, const double bottom, const ros::Time ros_time)
{
	auto createPoint = [](const double x, const double y, const double z)
	{
		geometry_msgs::Point32 tmp_p;
		tmp_p.x = x;
		tmp_p.y = y;
		tmp_p.z = z;
		return tmp_p;
	};

	geometry_msgs::PolygonStamped body_msg;
	body_msg.header.frame_id = baselink_frame_;
	body_msg.header.stamp = ros_time;
	body_msg.polygon.points.push_back(createPoint(front, left,  bottom));
	body_msg.polygon.points.push_back(createPoint(rear,  left,  bottom));
	body_msg.polygon.points.push_back(createPoint(rear,  right, bottom));
	body_msg.polygon.points.push_back(createPoint(front, right, bottom));
	body_msg.polygon.points.push_back(createPoint(front, left,  bottom));

	body_msg.polygon.points.push_back(createPoint(front, left,  top));

	body_msg.polygon.points.push_back(createPoint(rear,  left,  top));
	body_msg.polygon.points.push_back(createPoint(rear,  left,  bottom));
	body_msg.polygon.points.push_back(createPoint(rear,  left,  top));

	body_msg.polygon.points.push_back(createPoint(rear, right, top));
	body_msg.polygon.points.push_back(createPoint(rear, right, bottom));
	body_msg.polygon.points.push_back(createPoint(rear, right, top));

	body_msg.polygon.points.push_back(createPoint(front,  right, top));
	body_msg.polygon.points.push_back(createPoint(front,  right, bottom));
	body_msg.polygon.points.push_back(createPoint(front,  right, top));

	body_msg.polygon.points.push_back(createPoint(front, left,  top));

	body_polygon_pub_.publish(body_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "footprint_filter");
	FootprintFilter node;
	ros::spin();

	return 0;
}

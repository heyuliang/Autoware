/*
 *  Copyright (c) 2018, Nagoya University
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
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * cloud_projector_node.cpp
 *
 *  Created on: Nov 6th, 2018
 */

#ifndef PROJECT_CLOUD_PROJECTOR_H
#define PROJECT_CLOUD_PROJECTOR_H

#define __APP_NAME__ "cloud_projector"

#include <string>
#include <vector>
#include <chrono>

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Eigen>

class RosCloudProjectorApp
{
  ros::NodeHandle                     node_handle_;
  ros::Publisher                      publisher_projected_cloud_;
  ros::Subscriber                     intrinsics_subscriber_;

  tf2_ros::TransformListener          transform_listener_;
  tf2_ros::Buffer                     tf_buffer_;
  geometry_msgs::TransformStamped     camera_lidar_tf_;

  cv::Size                            image_size_;
  cv::Mat                             camera_instrinsics_;
  cv::Mat                             distortion_coefficients_;

  std::string                         image_frame_id_;

  bool                                processing_;
  bool                                camera_info_ok_;
  bool                                camera_lidar_tf_ok_;

  float                               fx_, fy_, cx_, cy_;

  ros::Subscriber                     cloud_subscriber_;

  pcl::PointXYZ TransformPoint(const pcl::PointXYZ &in_point, const geometry_msgs::TransformStamped &in_transform);

  void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg);

  geometry_msgs::TransformStamped
  FindTransform(const std::string &in_target_frame, const std::string &in_source_frame);

  void IntrinsicsCallback(const sensor_msgs::CameraInfo& in_message);

  void InitializeRosIo(ros::NodeHandle &in_private_handle);

public:
  void Run();
  RosCloudProjectorApp();
};

#endif //PROJECT_PIXEL_CLOUD_FUSION_H

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

#include "cloud_projector/cloud_projector.h"

pcl::PointXYZ
RosCloudProjectorApp::TransformPoint(const pcl::PointXYZ &in_point, const geometry_msgs::TransformStamped &in_transform)
{
  geometry_msgs::Point point, point_transformed;

  point.x = in_point.x;
  point.y = in_point.y;
  point.z = in_point.z;

  tf2::doTransform(point, point_transformed, in_transform);

  return pcl::PointXYZ(point_transformed.x, point_transformed.y, point_transformed.z);
}

void RosCloudProjectorApp::CloudCallback(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg)
{
  if (!camera_lidar_tf_ok_)
  {
    camera_lidar_tf_ = FindTransform(image_frame_id_, in_cloud_msg->header.frame_id);
  }
  if (!camera_info_ok_ || !camera_lidar_tf_ok_)
  {
    ROS_INFO("[%s] Waiting for Camera-Lidar TF and Intrinsics to be available.", __APP_NAME__);
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*in_cloud_msg, *in_cloud);

  std::vector<pcl::PointXYZ> cam_cloud(in_cloud->points.size());
  cv::Mat cloud_projected_image(image_size_.height, image_size_.width, CV_32FC1);
  cv::Mat normalized_output;

#pragma omp for
  for (size_t i = 0; i < in_cloud->points.size(); i++)
  {
    cam_cloud[i] = TransformPoint(in_cloud->points[i], camera_lidar_tf_);
    int u = int(cam_cloud[i].x * fx_ / cam_cloud[i].z + cx_);
    int v = int(cam_cloud[i].y * fy_ / cam_cloud[i].z + cy_);
    if ((u >= 0) && (u < image_size_.width)
        && (v >= 0) && (v < image_size_.height)
        && cam_cloud[i].z > 0
        && cam_cloud[i].z < 70
      )
    {
      cloud_projected_image.at<float>(cv::Point(u, v)) = cam_cloud[i].z;
    }
  }

  //normalize
  cv::normalize(cloud_projected_image, normalized_output, 255, 0, cv::NORM_MINMAX, CV_8UC1);

  // Publish
  cv_bridge::CvImage out_msg;
  out_msg.header = in_cloud_msg->header;
  out_msg.header.frame_id   = image_frame_id_;
  out_msg.encoding = sensor_msgs::image_encodings::MONO8;
  out_msg.image    = normalized_output; // Your cv::Mat

  publisher_projected_cloud_.publish(out_msg.toImageMsg());
}

void RosCloudProjectorApp::IntrinsicsCallback(const sensor_msgs::CameraInfo &in_message)
{
  image_size_.height = in_message.height;
  image_size_.width = in_message.width;

  camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 3; col++)
    {
      camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
    }
  }

  distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
  for (int col = 0; col < 5; col++)
  {
    distortion_coefficients_.at<double>(col) = in_message.D[col];
  }

  fx_ = static_cast<float>(in_message.P[0]);
  fy_ = static_cast<float>(in_message.P[5]);
  cx_ = static_cast<float>(in_message.P[2]);
  cy_ = static_cast<float>(in_message.P[6]);

  intrinsics_subscriber_.shutdown();
  camera_info_ok_ = true;
  image_frame_id_ = in_message.header.frame_id;
  ROS_INFO("[%s] CameraIntrinsics obtained.", __APP_NAME__);
}

geometry_msgs::TransformStamped
RosCloudProjectorApp::FindTransform(const std::string &in_target_frame, const std::string &in_source_frame)
{
  geometry_msgs::TransformStamped transform;

  camera_lidar_tf_ok_ = false;
  try
  {
    transform = tf_buffer_.lookupTransform(in_target_frame, in_source_frame, ros::Time(0));
    camera_lidar_tf_ok_ = true;
    ROS_INFO("[%s] Camera-Lidar TF obtained", __APP_NAME__);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
  }

  return transform;
}

void RosCloudProjectorApp::InitializeRosIo(ros::NodeHandle &in_private_handle)
{
  //get params
  std::string points_src, image_src, camera_info_src, projected_topic_str = "/image_depth";
  std::string name_space_str = ros::this_node::getNamespace();

  ROS_INFO("[%s] This node requires: Registered TF(Lidar-Camera), CameraInfo, Image, and PointCloud.", __APP_NAME__);
  in_private_handle.param<std::string>("points_src", points_src, "/points_raw");
  ROS_INFO("[%s] points_src: %s", __APP_NAME__, points_src.c_str());

  in_private_handle.param<std::string>("camera_info_src", camera_info_src, "/camera_info");
  ROS_INFO("[%s] camera_info_src: %s", __APP_NAME__, camera_info_src.c_str());

  if (name_space_str != "/")
  {
    if (name_space_str.substr(0, 2) == "//")
    {
      name_space_str.erase(name_space_str.begin());
    }
    image_src = name_space_str + image_src;
    projected_topic_str = name_space_str + projected_topic_str;
    camera_info_src = name_space_str + camera_info_src;
  }

  //generate subscribers and sychronizers
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, camera_info_src.c_str());
  intrinsics_subscriber_ = in_private_handle.subscribe(camera_info_src,
                                                       1,
                                                       &RosCloudProjectorApp::IntrinsicsCallback, this);

  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, points_src.c_str());
  cloud_subscriber_ = in_private_handle.subscribe(points_src,
                                                  1,
                                                  &RosCloudProjectorApp::CloudCallback, this);

  publisher_projected_cloud_ = node_handle_.advertise<sensor_msgs::Image>(projected_topic_str, 1);
  ROS_INFO("[%s] Publishing projected pointcloud in %s", __APP_NAME__, projected_topic_str.c_str());

}


void RosCloudProjectorApp::Run()
{
  ros::NodeHandle private_node_handle("~");

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener transform_listener(tf_buffer);

  InitializeRosIo(private_node_handle);

  ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

  ros::spin();

  ROS_INFO("[%s] END", __APP_NAME__);
}

RosCloudProjectorApp::RosCloudProjectorApp() : transform_listener_(tf_buffer_)
{
  camera_lidar_tf_ok_ = false;
  camera_info_ok_ = false;
  processing_ = false;
  image_frame_id_ = "";
}
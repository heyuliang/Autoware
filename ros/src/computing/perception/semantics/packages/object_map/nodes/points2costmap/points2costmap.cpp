/*
 *  Copyright (c) 2015, Nagoya University
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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include <utility>

namespace
{
// ros
ros::Publisher costmap_pub_;
// params
bool use_points_;
bool use_objects_;
double vehicle_length_;
double vehicle_width_;
double resolution_;
int cell_width_;
int cell_height_;
double offset_x_;
double offset_y_;
double offset_z_;
int occupied_cost_;
// variables
nav_msgs::OccupancyGrid costmap_;

void initCostmap()
{
  costmap_.info.resolution = resolution_;
  costmap_.info.width = cell_width_;
  costmap_.info.height = cell_height_;
  costmap_.info.origin.position.x = (-1) * (cell_width_ / 2.0) * resolution_ + offset_x_;
  costmap_.info.origin.position.y = (-1) * (cell_height_ / 2.0) * resolution_ + offset_y_;
  costmap_.info.origin.position.z = offset_z_;
  costmap_.info.origin.orientation.x = 0.0;
  costmap_.info.origin.orientation.y = 0.0;
  costmap_.info.origin.orientation.z = 0.0;
  costmap_.info.origin.orientation.w = 1.0;
  costmap_.data.resize(cell_width_ * cell_height_);
}

void updateCostMap(const pcl::PointCloud<pcl::PointXYZ>& points)
{
  std::fill(costmap_.data.begin(), costmap_.data.end(), 0);
  double map_center_x = (cell_width_ / 2.0) * resolution_ - offset_x_;
  double map_center_y = (cell_height_ / 2.0) * resolution_ - offset_y_;

  for (const auto& p : points.points)
  {
    if (std::fabs(p.x) < vehicle_length_ && std::fabs(p.y) < vehicle_width_)
    {
      continue;
    }

    int grid_x = (p.x + map_center_x) / resolution_;
    int grid_y = (p.y + map_center_y) / resolution_;

    if (grid_x < 0 || grid_x >= cell_width_ || grid_y < 0 || grid_y >= cell_height_)
    {
      continue;
    }

    int index = cell_width_ * grid_y + grid_x;
    costmap_.data[index] = occupied_cost_;
  }
}

void pointsCallback(const sensor_msgs::PointCloud2& msg)
{
  if (!use_points_)
  {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> points;
  pcl::fromROSMsg(msg, points);

  costmap_.header = msg.header;
  updateCostMap(points);
  costmap_pub_.publish(costmap_);
}

void objectsCallback(const autoware_msgs::DetectedObjectArray& msg)
{
  if (!use_objects_)
  {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> points;
  for (const auto& obj : msg.objects)
  {
    if (obj.pointcloud.data.size() != 0)
    {
      pcl::PointCloud<pcl::PointXYZ> p;
      pcl::fromROSMsg(obj.pointcloud, p);
      points += p;
    }
  }

  if (points.size() > 0)
  {
    costmap_.header = msg.objects[0].header;
    updateCostMap(points);
    costmap_pub_.publish(costmap_);
  }
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "points2costmap");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param<bool>("use_points", use_points_, true);
  private_nh.param<bool>("use_objects", use_objects_, true);
  private_nh.param<double>("vehicle_length", vehicle_length_, 4.5);
  private_nh.param<double>("vehicle_width", vehicle_width_, 1.75);
  private_nh.param<double>("resolution", resolution_, 0.2);
  private_nh.param<int>("cell_width", cell_width_, 300);
  private_nh.param<int>("cell_height", cell_height_, 150);
  private_nh.param<double>("offset_x", offset_x_, 25.0);
  private_nh.param<double>("offset_y", offset_y_, 0.0);
  private_nh.param<double>("offset_z", offset_z_, -2.0);
  private_nh.param<int>("points_cost", occupied_cost_, 100);

  costmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("realtime_cost_map", 10);
  ros::Subscriber points_sub = nh.subscribe("points_no_ground", 1, pointsCallback);
  ros::Subscriber objects_sub = nh.subscribe("detected_objects", 1, objectsCallback);

  initCostmap();

  ros::spin();
}

#ifndef lidar_adas_ground_detect_H
#define lidar_adas_ground_detect_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

class AdasGroundDetect
{
public:
  AdasGroundDetect();
  void run();
private:
  grid_map::GridMap adas_gridmap_;
  ros::NodeHandle node_handle_;
  ros::Subscriber sub_points_;
  ros::Subscriber sub_adas_gridmap_;
  ros::Publisher  pub_elevated_points_;
  void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud);
  void adasGridmapCallback(const grid_map_msgs::GridMap& in_adas_grid);
  // bool isPointInGridAndNonGround(const grid_map::GridMap& in_grid_map, const cv::Mat& in_grid_image,
  //                       const pcl::PointXYZ &pcl_point);
  bool isPointInGridAndNonGround(const grid_map::GridMap& in_grid_map, const grid_map::Matrix& in_grid_data,
                        const pcl::PointXYZ &pcl_point);
};

#endif  // lidar_adas_ground_detect_H

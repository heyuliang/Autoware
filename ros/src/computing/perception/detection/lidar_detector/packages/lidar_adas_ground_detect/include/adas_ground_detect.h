#ifndef lidar_adas_ground_detect_H
#define lidar_adas_ground_detect_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
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

  tf::TransformListener tf_listener_;
  void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud);
  void adasGridmapCallback(const grid_map_msgs::GridMap& in_adas_grid);
  bool isPointInGridAndNonGround(const grid_map::GridMap& in_grid_map,
                                 const grid_map::Matrix& in_grid_data,
                                 const pcl::PointXYZ &pcl_point);
  tf::StampedTransform makeTransform(const std::string &in_target_frame,
                                     const std::string &in_source_frame,
                                     const tf::TransformListener &in_tf_listener);
  pcl::PointXYZ makeTransformedPoint(const pcl::PointXYZ &in_pcl_point,
                               const tf::Transform &in_tf);
};

#endif  // lidar_adas_ground_detect_H



#include "adas_ground_detect.h"

AdasGroundDetect::AdasGroundDetect()
{

}

void AdasGroundDetect::run()
{
  sub_adas_gridmap_ = node_handle_.subscribe("/grid_map_wayarea", 1, &AdasGroundDetect::adasGridmapCallback, this);
  sub_points_       = node_handle_.subscribe("/points_raw", 1, &AdasGroundDetect::pointsCallback, this);
  pub_elevated_points_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/points_no_ground", 1);
}

// bool AdasGroundDetect::isPointInGridAndNonGround(const grid_map::GridMap& in_grid_map, const cv::Mat& in_grid_image,
//                       const pcl::PointXYZ &pcl_point)
// {
//   // calculate out_grid_map position
//   grid_map::Position map_pos = in_grid_map.getPosition();
//   double origin_x_offset = in_grid_map.getLength().x() / 2.0 - map_pos.x();
//   double origin_y_offset = in_grid_map.getLength().y() / 2.0 - map_pos.y();
//   // coordinate conversion for cv image
//   double cv_x = (in_grid_map.getLength().y() - origin_y_offset - pcl_point.y) / in_grid_map.getResolution();
//   double cv_y = (in_grid_map.getLength().x() - origin_x_offset - pcl_point.x) / in_grid_map.getResolution();
//
//   // check coords are inside the gridmap
//   if (cv_x < 0 || cv_x > in_grid_image.cols || cv_y < 0 || cv_y > in_grid_image.rows)
//   {
//     return false;
//   }
//
//   std::cout << "compare height "<< in_grid_image.at<float>(cv_y, cv_x) << std::endl;
//   if (pcl_point.z  > in_grid_image.at<float>(cv_y, cv_x))
//   {
//     return true;
//   }
//   return false;
// }

bool AdasGroundDetect::isPointInGridAndNonGround(const grid_map::GridMap& in_grid_map, const grid_map::Matrix& in_grid_data,
                      const pcl::PointXYZ &pcl_point)
{
  // calculate out_grid_map position
  grid_map::Position map_pos = in_grid_map.getPosition();
  double origin_x_offset = in_grid_map.getLength().x() / 2.0 - map_pos.x();
  double origin_y_offset = in_grid_map.getLength().y() / 2.0 - map_pos.y();
  // coordinate conversion for cv image
  double cv_x = (in_grid_map.getLength().y() - origin_y_offset - pcl_point.y) / in_grid_map.getResolution();
  double cv_y = (in_grid_map.getLength().x() - origin_x_offset - pcl_point.x) / in_grid_map.getResolution();

  // check coords are inside the gridmap
  // TODO: remove magic number
  if (cv_x < 0 || cv_x > 750 || cv_y < 0 || cv_y > 750)
  {
    return false;
  }

  std::cout << "compare height "<< in_grid_data(cv_y, cv_x) << std::endl;
  if (pcl_point.z  > (in_grid_data(cv_y, cv_x) + 0.25))
  {
    return true;
  }
  return false;
}

void AdasGroundDetect::pointsCallback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_pointcloud)
{
  pcl::PointCloud<pcl::PointXYZ> in_pcl_pointcloud;
  pcl::PointCloud<pcl::PointXYZ> out_pcl_pointcloud;
  pcl::fromROSMsg(*in_sensor_pointcloud, in_pcl_pointcloud);

  if (adas_gridmap_.exists("lanearea"))
  {
    std::cout << "callbalc "<<std::endl;
    // check if centroids are inside the drivable area
    cv::Mat grid_image;
    grid_map::GridMapCvConverter::toImage<float, 1>(adas_gridmap_,"lanearea",CV_32FC1,
                                                    -10,  10, grid_image);
    // std::cout << grid_image << std::endl;
    grid_map::Matrix grid_data = adas_gridmap_["lanearea"];
    for (unsigned int i = 0; i < in_pcl_pointcloud.size(); i++)
    {

      pcl::PointXYZ pcl_point = in_pcl_pointcloud[i];
      bool point_in_grid = isPointInGridAndNonGround(adas_gridmap_, grid_data, pcl_point);
      if(point_in_grid)
      {
        out_pcl_pointcloud.push_back(pcl_point);
      }
    }
    sensor_msgs::PointCloud2 out_sensor_pointcloud;
    pcl::toROSMsg(out_pcl_pointcloud, out_sensor_pointcloud);
    out_sensor_pointcloud.header = in_sensor_pointcloud->header;
    // std::cout << "out header "<< out_sensor_pointcloud.header <<std::endl;
    pub_elevated_points_.publish(out_sensor_pointcloud);
  }
  else
  {
    ROS_INFO("%s layer not contained in the OccupancyGrid", "lanearea");
  }
}

void AdasGroundDetect::adasGridmapCallback(const grid_map_msgs::GridMap& in_adas_grid)
{
  grid_map::GridMapRosConverter::fromMessage(in_adas_grid, adas_gridmap_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_adas_ground_detect");
  AdasGroundDetect node;
  node.run();
  ros::spin();

  return 0;
}

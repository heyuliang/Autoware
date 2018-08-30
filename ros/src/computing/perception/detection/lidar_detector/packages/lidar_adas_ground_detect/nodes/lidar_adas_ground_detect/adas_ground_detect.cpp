

#include "adas_ground_detect.h"

AdasGroundDetect::AdasGroundDetect()
{

}

void AdasGroundDetect::run()
{
  sub_adas_gridmap_ = node_handle_.subscribe("/adas_grid_map", 1, &AdasGroundDetect::adasGridmapCallback, this);
  sub_points_       = node_handle_.subscribe("/points_raw", 1, &AdasGroundDetect::pointsCallback, this);
  pub_elevated_points_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/points_no_ground", 1);
}

tf::StampedTransform AdasGroundDetect::makeTransform(const std::string &in_target_frame,
                                                     const std::string &in_source_frame,
                                                     const tf::TransformListener &in_tf_listener)
{
  tf::StampedTransform transform;

  try
  {
    in_tf_listener.lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  return transform;
}

pcl::PointXYZ AdasGroundDetect::makeTransformedPoint(const pcl::PointXYZ &in_pcl_point, const tf::Transform &in_tf)
{
  geometry_msgs::Point in_point;
  in_point.x = in_pcl_point.x;
  in_point.y = in_pcl_point.y;
  in_point.z = in_pcl_point.z;

  // std::cout << "velo pcl " << in_point << std::endl;
  tf::Point tf_point;
  tf::pointMsgToTF(in_point, tf_point);

  tf_point = in_tf * tf_point;

  geometry_msgs::Point out_point;
  tf::pointTFToMsg(tf_point, out_point);

  // std::cout << "world pcl " << out_point << std::endl;

  pcl::PointXYZ out_pcl_point;
  out_pcl_point.x = out_point.x;
  out_pcl_point.y = out_point.y;
  out_pcl_point.z = out_point.z;

  return out_pcl_point;
}

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

  // std::cout << "compare height "<< in_grid_data(cv_y, cv_x) << std::endl;
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
    // cv::Mat grid_image;
    // grid_map::GridMapCvConverter::toImage<float, 1>(adas_gridmap_,"lanearea",CV_32FC1,
    //                                                 -10,  10, grid_image);
    // std::cout << grid_image << std::endl;
    tf::StampedTransform tf = makeTransform("world", "velodyne", tf_listener_);
    grid_map::Matrix grid_data = adas_gridmap_["lanearea"];
    std::cout << "points size " << in_pcl_pointcloud.size() << std::endl;
    for (unsigned int i = 0; i < in_pcl_pointcloud.size(); i++)
    {

      pcl::PointXYZ pcl_point = in_pcl_pointcloud[i];
      pcl::PointXYZ transformed_pcl_point = makeTransformedPoint(pcl_point, tf);
      // std::cout << "world pcl " << transformed_pcl_point << std::endl;

      bool is_point_in_grid = isPointInGridAndNonGround(adas_gridmap_, grid_data, transformed_pcl_point);
      if(is_point_in_grid)
      {
        out_pcl_pointcloud.push_back(pcl_point);
      }
      // out_pcl_pointcloud.push_back(transformed_pcl_point);
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

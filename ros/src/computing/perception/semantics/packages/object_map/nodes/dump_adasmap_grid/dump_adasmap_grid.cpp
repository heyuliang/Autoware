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
 ********************/

#include <pcl/io/pcd_io.h>
#include "dump_adasmap_grid.h"

namespace object_map
{
WayareaToGrid::WayareaToGrid() : private_node_handle_("~")
{
  InitializeRosIo();
  use_pcd_map_ = true;
  // preciseGroundEstimationWithPCD();
}

std::vector<geometry_msgs::Point> WayareaToGrid::generateRectangleFromLaneInfo(LaneInfo lf)
{
  double forward_dx  = (lf.forward_point.x - lf.point.x )*3/2;
  double forward_dy  = (lf.forward_point.y - lf.point.y )*3/2;
  double backward_dx = (lf.point.x- lf.backward_point.x)*3/2;
  double backward_dy = (lf.point.y- lf.backward_point.y )*3/2;

  double direction = atan2(lf.forward_point.y - lf.point.y, lf.forward_point.x - lf.point.x);

  // direction toward left lane width
  double left_lane_direction = direction + M_PI/2;
  while (left_lane_direction > M_PI)
    left_lane_direction -= 2. * M_PI;
  while (left_lane_direction < -M_PI)
    left_lane_direction += 2. * M_PI;

  double right_lane_direction = direction - M_PI/2;
  while (right_lane_direction > M_PI)
    right_lane_direction -= 2. * M_PI;
  while (right_lane_direction < -M_PI)
    right_lane_direction += 2. * M_PI;


  double left_lane_x = cos(left_lane_direction)*lf.left_width + lf.point.x;
  double left_lane_y = sin(left_lane_direction)*lf.left_width + lf.point.y;


  double right_lane_x = cos(right_lane_direction)*lf.right_width + lf.point.x;
  double right_lane_y = sin(right_lane_direction)*lf.right_width + lf.point.y;


  geometry_msgs::Point left_top;
  left_top.x = left_lane_x + forward_dx;
  left_top.y = left_lane_y + forward_dy;
  left_top.z = lf.point.z;
  geometry_msgs::Point right_top;
  right_top.x = right_lane_x + forward_dx;
  right_top.y = right_lane_y + forward_dy;
  right_top.z = lf.point.z;
  geometry_msgs::Point right_bottom;
  right_bottom.x = right_lane_x - backward_dx;
  right_bottom.y = right_lane_y - backward_dy;
  right_bottom.z = lf.point.z;
  geometry_msgs::Point left_bottom;
  left_bottom.x = left_lane_x - backward_dx;
  left_bottom.y = left_lane_y - backward_dy;
  left_bottom.z = lf.point.z;

  std::vector<geometry_msgs::Point> rectangle_points{left_top, right_top, right_bottom, left_bottom};
  return rectangle_points;
}

std::vector<std::vector<geometry_msgs::Point>>
WayareaToGrid::generateLaneAreaPointsFromLaneInfo(std::vector<LaneInfo>& lane_info_vec)
{
  std::vector<std::vector<geometry_msgs::Point>> lane_area_points;
  for(size_t i = 0; i < lane_info_vec.size(); i++)
  {
    std::vector<geometry_msgs::Point> rectangle =
      generateRectangleFromLaneInfo(lane_info_vec[i]);
    lane_area_points.push_back(rectangle);
  }
  return lane_area_points;
}

void WayareaToGrid::loadLaneInfoFromVectorMap(ros::NodeHandle& in_private_node_handle,
                                std::vector<LaneInfo>& lane_info_vec)
{
  vector_map::VectorMap vmap;
  vmap.subscribe(in_private_node_handle,
                 vector_map::Category::POINT | vector_map::Category::DTLANE |
                 vector_map::Category::LANE, 10);

  std::vector<vector_map_msgs::Lane> lanes =
      vmap.findByFilter([](const vector_map_msgs::Lane &lane)
                        {
                          return true;
                        });

  if (lanes.empty())
  {
    ROS_WARN_STREAM("No Lane...");
    return;
  }

  int debug_count = 0;
  for (const auto &lane : lanes)
  {
    vector_map_msgs::DTLane dtlane = vmap.findByKey(vector_map::Key<vector_map::DTLane>(lane.did));
    vector_map_msgs::Lane forward_lane = vmap.findByKey(vector_map::Key<vector_map::Lane>(lane.flid));
    vector_map_msgs::Lane backward_lane = vmap.findByKey(vector_map::Key<vector_map::Lane>(lane.blid));
    vector_map_msgs::DTLane forward_dtlane = vmap.findByKey(vector_map::Key<vector_map::DTLane>(forward_lane.did));
    vector_map_msgs::DTLane backward_dtlane = vmap.findByKey(vector_map::Key<vector_map::DTLane>(backward_lane.did));

    if(lane.flid == 0 || lane.blid == 0)
    {
      continue;
    }

    vector_map_msgs::Point point = vmap.findByKey(vector_map::Key<vector_map::Point>(dtlane.pid));
    vector_map_msgs::Point forward_point = vmap.findByKey(vector_map::Key<vector_map::Point>(forward_dtlane.pid));
    vector_map_msgs::Point backward_point = vmap.findByKey(vector_map::Key<vector_map::Point>(backward_dtlane.pid));

    LaneInfo li;
    li.point = vector_map::convertPointToGeomPoint(point);
    li.forward_point = vector_map::convertPointToGeomPoint(forward_point);
    li.backward_point = vector_map::convertPointToGeomPoint(backward_point);
    li.left_width  = dtlane.lw;
    li.right_width = dtlane.rw;
    lane_info_vec.push_back(li);
    // if(debug_count < 5)
    // {
    //   std::cout << "lane did " << lane.did << std::endl;
    //   std::cout << "forward lane did "<<forward_lane.did << std::endl;
    //   std::cout << "backward lane did "<<backward_lane.did << std::endl;
    //   std::cout << "forward point "<< li.forward_point << std::endl;
    //   std::cout << "backward point "<< li.backward_point << std::endl;
    // }
    debug_count++;
    // std::cout << "lane did " << lane.did << std::endl;
    // return;
  }
}

void WayareaToGrid::InitializeRosIo()
{
  private_node_handle_.param<std::string>("world_frame", world_frame_, "world");
  private_node_handle_.param<std::string>("map_frame", map_frame_, "map");
  private_node_handle_.param<double>("grid_resolution", grid_resolution_, 0.2);
  private_node_handle_.param<double>("grid_length_x", grid_length_x_, 150);
  private_node_handle_.param<double>("grid_length_y", grid_length_y_, 150);
  private_node_handle_.param<double>("grid_position_x", grid_position_x_, 20);
  private_node_handle_.param<double>("grid_position_x", grid_position_y_, 0);

  publisher_grid_map_ = node_handle_.advertise<grid_map_msgs::GridMap>("adas_grid_map", 1, true);
  publisher_occupancy_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("occupancy_wayarea", 1, true);
  pub_point_ = node_handle_.advertise<sensor_msgs::PointCloud2>("grid_map_point", 1, true);
}

void WayareaToGrid::publishPointcloud()
{
  sensor_msgs::PointCloud2 pointCloud;
  grid_map::GridMapRosConverter::toPointCloud(gridmap_, "lanearea", pointCloud);

  pub_point_.publish(pointCloud);
}

std::vector<std::string> WayareaToGrid::globFilesInDirectory(const std::string& pattern)
{
  glob_t glob_result;
  glob(pattern.c_str(),GLOB_TILDE,NULL,&glob_result);
  std::vector<std::string> files;
  for(unsigned int i=0;i<glob_result.gl_pathc;++i){
      files.push_back(std::string(glob_result.gl_pathv[i]));
  }
  globfree(&glob_result);
  return files;
}

pcl::PointXYZ WayareaToGrid::makeTransformedPoint(const pcl::PointXYZ &in_pcl_point)
{
  geometry_msgs::Point in_point;
  in_point.x = in_pcl_point.x;
  in_point.y = in_pcl_point.y;
  in_point.z = in_pcl_point.z;

  // std::cout << "velo pcl " << in_point << std::endl;
  tf::Point tf_point;
  tf::pointMsgToTF(in_point, tf_point);

  tf_point = transform_ * tf_point;

  geometry_msgs::Point out_point;
  tf::pointTFToMsg(tf_point, out_point);

  // std::cout << "world pcl " << out_point << std::endl;

  pcl::PointXYZ out_pcl_point;
  out_pcl_point.x = out_point.x;
  out_pcl_point.y = out_point.y;
  out_pcl_point.z = out_point.z;

  return out_pcl_point;
}

std::vector<double> WayareaToGrid::makeGridPointIndex(const pcl::PointXYZ& pcl_point)
{
  // calculate out_grid_map position
  grid_map::Position map_pos = gridmap_.getPosition();
  double origin_x_offset = gridmap_.getLength().x() / 2.0 - map_pos.x();
  double origin_y_offset = gridmap_.getLength().y() / 2.0 - map_pos.y();
  // coordinate conversion for cv image
  // pcl::PointXYZ grid_point_index;
  double cv_x = (gridmap_.getLength().y() - origin_y_offset - pcl_point.y) / gridmap_.getResolution();
  double cv_y = (gridmap_.getLength().x() - origin_x_offset - pcl_point.x) / gridmap_.getResolution();
  // std::cout << "make index index " << cv_x << " "<<cv_y << std::endl;
  std::vector<double> grid_point_index;
  grid_point_index.push_back(cv_x);
  grid_point_index.push_back(cv_y);
  // std::cout << "make index index " << grid_point_index[0] << " "<< grid_point_index[1] << std::endl;
  return grid_point_index;
}

double WayareaToGrid::fetchGridHeightFromPoint(pcl::PointXYZ transformed_point,
                                               const grid_map::Matrix& grid_data,
                                               const std::vector<double> &grid_point_ind)
{
  // std::vector<double> grid_point_ind = makeGridPointIndex(transformed_point);
  double cv_x = grid_point_ind[0];
  double cv_y = grid_point_ind[1];
  return grid_data(cv_y, cv_x);
}

bool WayareaToGrid::isPointInGrid(const pcl::PointXYZ& pcl_point, const std::vector<double> &grid_point_ind)
{
  // std::vector<double> grid_point_ind = makeGridPointIndex(pcl_point);
  double cv_x = grid_point_ind[0];
  double cv_y = grid_point_ind[1];
  // std::cout << "index " << cv_x << " "<<cv_y << std::endl;
  if (cv_x < 0 || cv_x >= 750 || cv_y < 0 || cv_y >= 750)
  {
    return false;
  }
  return true;
}

void WayareaToGrid::updateGridHeight(double  pcl_height,
                                     const std::vector<double> &grid_point_ind,
                                     grid_map::Matrix& grid_data)
{
  // std::vector<double> grid_point_ind = makeGridPointIndex(pcl_point);
  double cv_x = grid_point_ind[0];
  double cv_y = grid_point_ind[1];
  grid_data(cv_y, cv_x) = pcl_height;
}

void WayareaToGrid::updateGridmapWithPointcloud(pcl::PointCloud<pcl::PointXYZ> partial_pointcloud)
{
  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  grid_map::Matrix grid_data = gridmap_["lanearea"];
  // std::cout << "partial pc size "<< partial_pointcloud.size() << std::endl;
  // vector<vector<vector<double> > > vec_record_points_height (
  //                   grid_length_x_*(1/grid_resolution_),
  //                   vector<vector<double> >(grid_length_y_*(1/grid_resolution_),
  //                   vector <double>);


  std::vector<double> height_vec ;
  std::vector<std::vector<double>> vec_y_height(grid_length_y_*(1/grid_resolution_) , height_vec) ;
  std::vector<std::vector<std::vector<double>>> vec_x_y_height(grid_length_x_*(1/grid_resolution_), vec_y_height) ;
  // std::vector<std::vector<double> > vec_record_points_height(
  //   grid_length_x_*(1/grid_resolution_),
  //   std::vector<double>(grid_length_y_*(1/grid_resolution_)));
  for(size_t i = 0; i < partial_pointcloud.size(); i++)
  {
    pcl::PointXYZ transformed_point = makeTransformedPoint(partial_pointcloud[i]);
    // std::cout << "finished transform" << std::endl;

    std::vector<double> grid_point_ind = makeGridPointIndex(transformed_point);
    bool is_inside_range = isPointInGrid(transformed_point, grid_point_ind);
    // std::cout << "finished ispoint in" << std::endl;
    if(is_inside_range)
    {
      // double height = fetchGridHeightFromPoint(transformed_point, grid_data, grid_point_ind);
      // if(std::abs(height - transformed_point.z) < 0.5)
      // {
      //   double pcl_height = transformed_point.z;

      // std::cout << "ind " << grid_point_ind[0] << " " << grid_point_ind[1] << std::endl;
      // if(transformed_point.z < -2.0)
      // {
      //   continue;
      // }

      if(transformed_point.z < -0.15)
      {
        continue;
      }
      vec_x_y_height[std::floor(grid_point_ind[0])]
                              [std::floor(grid_point_ind[1])]
                              .push_back(transformed_point.z);

        // updateGridHeight(transformed_point.z, grid_point_ind, grid_data);
      // }
    }
    // transformed_pointcloud.push_back(transformed_point);
  }

  for(size_t i = 0; i < vec_x_y_height.size(); i++)
  {
    for(size_t j = 0; j < vec_x_y_height[0].size(); j++)
    {
      // for(size_t k = 0; k < vec_x_y_height[i][j].size(); k ++)
      // {
        size_t size = vec_x_y_height[i][j].size();
        if(size == 0)
        {
          continue;
        }
        else
        {
          // double median_height = 0;
          // std::sort(vec_x_y_height[i][j].begin(),
          //           vec_x_y_height[i][j].end());
          // if (size % 2 == 0)
          // {
          //   median_height = (vec_x_y_height[i][j][size / 2 - 1] +
          //                   vec_x_y_height[i][j][size / 2]) / 2;
          // }
          // else
          // {
          //   median_height = vec_x_y_height[i][j][size / 2];
          // }

          double min_height = *std::min_element(vec_x_y_height[i][j].begin(),
                                               vec_x_y_height[i][j].end());
          std::vector<double> ind;
          ind.push_back(i);
          ind.push_back(j);

          // updateGridHeight(median_height, ind, grid_data);
          updateGridHeight(min_height, ind, grid_data);
        // }
      }
    }
  }

  gridmap_["lanearea"] = grid_data;
}

void WayareaToGrid::preciseGroundEstimationWithPCD()
{
  // std::cout << "calling file func" << std::endl;
  std::vector<std::string> file_paths = globFilesInDirectory("/home/kosuke/data/shimz/pointcloud_map/orig/*");
  for (size_t i = 0; i < file_paths.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZ> partial_pointcloud;
    std::cout << "loading pcd file " << std::endl;
    std::cout << file_paths[i] << std::endl;
    pcl::io::loadPCDFile<pcl::PointXYZ> (file_paths[i], partial_pointcloud);
    std::cout << "finished load pcd file " << std::endl;
    // bool is_inside_range = isPointInGrid(const pcl::PointXYZ &pcl_point);
    updateGridmapWithPointcloud(partial_pointcloud);
  }
}

void WayareaToGrid::Run()
{
  // load necessary vector map info
  std::cout << "start loading gridmap" << std::endl;
  std::vector<LaneInfo> lane_info_vec;
  LoadRoadAreasFromVectorMap(private_node_handle_, area_points_);
  loadLaneInfoFromVectorMap(private_node_handle_, lane_info_vec);
  std::cout << "finished loading gridmap!" << std::endl;

  // augment lane point so that they form polygons
  std::vector<std::vector<geometry_msgs::Point>> lane_area_points =
        generateLaneAreaPointsFromLaneInfo(lane_info_vec);

  bool set_map = false;
  ros::Rate loop_rate(0.01);

  while (ros::ok())
  {
    if (!set_map)
    {
      gridmap_.add("lanearea");
      gridmap_.setFrameId(world_frame_);
      gridmap_.setGeometry(grid_map::Length(grid_length_x_, grid_length_y_), grid_resolution_,
                           grid_map::Position(grid_position_x_, grid_position_y_));
      set_map = true;
    }

    // timer start
    // auto start = std::chrono::system_clock::now();

    if (!lane_area_points.empty())
    {
      std::cout << "start making gridmap" << std::endl;
      // FillPolygonLaneAreas(gridmap_, transform_, lane_area_points, "lanearea", 20, -10,
      //                  10, "world", map_frame_, tf_listener_);

      if(use_pcd_map_)
      {
        gridmap_["lanearea"].setConstant(20);
        transform_ = FindTransform("world", map_frame_, tf_listener_);
        preciseGroundEstimationWithPCD();
      }

      PublishGridMap(gridmap_, publisher_grid_map_);
      PublishOccupancyGrid(gridmap_, publisher_occupancy_, "lanearea", -10, 10);
      publishPointcloud();
      std::cout << "finished making gridmap!" << std::endl;

    }
    // timer end
    // auto end = std::chrono::system_clock::now();
    // auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;

    loop_rate.sleep();
  }
}

}  // namespace object_map

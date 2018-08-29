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

#include "wayarea2grid.h"

namespace object_map
{
WayareaToGrid::WayareaToGrid() : private_node_handle_("~")
{
  InitializeRosIo();
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
    // return lane_area_points;
    // if(i == 10)
    // {
    //   return lane_area_points;
    // }
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

    // vector_map_msgs::DTLane node = vmap.findByKey(vector_map::Key<vector_map::Node>(lane.bnid));
    // vector_map_msgs::Lane forward_lane = vmap.findByKey(vector_map::Key<vector_map::Lane>(lane.flid));
    // vector_map_msgs::Lane backward_lane = vmap.findByKey(vector_map::Key<vector_map::Lane>(lane.blid));
    // vector_map_msgs::DTLane forward_dtlane = vmap.findByKey(vector_map::Key<vector_map::DTLane>(forward_lane.did));
    // vector_map_msgs::DTLane backward_dtlane = vmap.findByKey(vector_map::Key<vector_map::DTLane>(backward_lane.did));


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
    if(debug_count < 5)
    {
      std::cout << "lane did " << lane.did << std::endl;
      std::cout << "forward lane did "<<forward_lane.did << std::endl;
      std::cout << "backward lane did "<<backward_lane.did << std::endl;
      std::cout << "forward point "<< li.forward_point << std::endl;
      std::cout << "backward point "<< li.backward_point << std::endl;
    }
    debug_count++;
    // std::cout << "lane did " << lane.did << std::endl;
    // return;
  }
}

// d::loadLaneInfoFromVectorMap(ros::NodeHandle& in_private_node_handle,
//                                 std::vector<LaneInfo>& lane_info_vec)
// {
//   vector_map::VectorMap vmap;
//   vmap.subscribe(in_private_node_handle,
//                  vector_map::Category::POINT | vector_map::Category::DTLANE |
//                  vector_map::Category::LANE, 10);
//
//   std::vector<vector_map_msgs::Lane> lanes =
//       vmap.findByFilter([](const vector_map_msgs::Lane &lane)
//                         {
//                           return true;
//                         });
//
//   if (lanes.empty())
//   {
//     ROS_WARN_STREAM("No Lane...");
//     return;
//   }
//
//   int debug_count = 0;
//   for (const auto &lane : lanes)
//   {
//     vector_map_msgs::Node backward_node = vmap.findByKey(vector_map::Key<vector_map::Node>(lane.bnid));
//     vector_map_msgs::Node forward_node = vmap.findByKey(vector_map::Key<vector_map::Node>(lane.fnid));
//
//
//     // vector_map_msgs::Point point = vmap.findByKey(vector_map::Key<vector_map::Point>(dtlane.pid));
//     vector_map_msgs::Point backward_point = vmap.findByKey(vector_map::Key<vector_map::Point>(backward_node.pid));
//     vector_map_msgs::Point forward_point = vmap.findByKey(vector_map::Key<vector_map::Point>(forward_node.pid));
//
//     LaneInfo li;
//     // li.point = vector_map::convertPointToGeomPoint(point);
//     li.backward_point = vector_map::convertPointToGeomPoint(backward_point);
//     li.forward_point = vector_map::convertPointToGeomPoint(forward_point);
//     li.left_width  = dtlane.lw;
//     li.right_width = dtlane.rw;
//     lane_info_vec.push_back(li);
//     if(debug_count < 5)
//     {
//       std::cout << "lane did " << lane.did << std::endl;
//       std::cout << "forward lane did "<<forward_lane.did << std::endl;
//     }
//     debug_count++;
//     // std::cout << "lane did " << lane.did << std::endl;
//     // return;
//   }
// }

void WayareaToGrid::InitializeRosIo()
{
  private_node_handle_.param<std::string>("sensor_frame", sensor_frame_, "velodyne");
  private_node_handle_.param<std::string>("map_frame", map_frame_, "map");
  private_node_handle_.param<double>("grid_resolution", grid_resolution_, 0.2);
  private_node_handle_.param<double>("grid_length_x", grid_length_x_, 80);
  private_node_handle_.param<double>("grid_length_y", grid_length_y_, 30);
  private_node_handle_.param<double>("grid_position_x", grid_position_x_, 20);
  private_node_handle_.param<double>("grid_position_x", grid_position_y_, 0);

  publisher_grid_map_ = node_handle_.advertise<grid_map_msgs::GridMap>("grid_map_wayarea", 1, true);
  publisher_occupancy_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("occupancy_wayarea", 1, true);
}

void WayareaToGrid::Run()
{
  // load necessary vector map info
  std::vector<LaneInfo> lane_info_vec;
  LoadRoadAreasFromVectorMap(private_node_handle_, area_points_);
  loadLaneInfoFromVectorMap(private_node_handle_, lane_info_vec);

  // augment lane point so that they form polygons
  std::vector<std::vector<geometry_msgs::Point>> lane_area_points =
        generateLaneAreaPointsFromLaneInfo(lane_info_vec);

  bool set_map = false;
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    if (!set_map)
    {
      gridmap_.add(grid_layer_name_);
      gridmap_.setFrameId(sensor_frame_);
      gridmap_.setGeometry(grid_map::Length(grid_length_x_, grid_length_y_), grid_resolution_,
                           grid_map::Position(grid_position_x_, grid_position_y_));
      gridmap_.add("lanearea");
      set_map = true;
    }

    // timer start
    // auto start = std::chrono::system_clock::now();

    // if (!area_points_.empty())
    // {
    //   FillPolygonAreas(gridmap_, area_points_, grid_layer_name_, OCCUPANCY_NO_ROAD, OCCUPANCY_ROAD, grid_min_value_,
    //                    grid_max_value_, sensor_frame_, map_frame_, tf_listener_);
    //   PublishGridMap(gridmap_, publisher_grid_map_);
    //   PublishOccupancyGrid(gridmap_, publisher_occupancy_, grid_layer_name_, grid_min_value_, grid_max_value_);
    // }

    if (!lane_area_points.empty())
    {
      FillPolygonLaneAreas(gridmap_, lane_area_points, "lanearea", -10, -10,
                       10, sensor_frame_, map_frame_, tf_listener_);
      PublishGridMap(gridmap_, publisher_grid_map_);
      PublishOccupancyGrid(gridmap_, publisher_occupancy_, "lanearea", -10, 10);
    }

    // PublishGridMap(gridmap_, publisher_grid_map_);
    // PublishOccupancyGrid(gridmap_, publisher_occupancy_, "lanearea", -10, 10);

    // timer end
    // auto end = std::chrono::system_clock::now();
    // auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;

    loop_rate.sleep();
  }
}

}  // namespace object_map

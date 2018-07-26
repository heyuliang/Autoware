/*
 *  Copyright (c) 2018, TierIV, Inc.

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
#include "waypoint_manager.h"

namespace waypoint_maker
{

WaypointManager::WaypointManager() : lane_idx_(0), replanning_mode_(false), pstop_distance_(6.0)
{
  lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>("/based/lane_waypoints_array", 10, true);
  lane_sub_ = nh_.subscribe("/based/lane_waypoints_raw", 1, &WaypointManager::laneCallback, this);
  state_sub_ = nh_.subscribe("/decision_maker/state", 1, &WaypointManager::stateCallback, this);
  config_sub_ = nh_.subscribe("/config/waypoint_manager", 1, &WaypointManager::configCallback, this);
  wfconfig_sub_ = nh_.subscribe("/config/waypoint_follower", 1, &WaypointManager::wfConfigCallback, this);
}

WaypointManager::~WaypointManager()
{
}

void WaypointManager::devideLane(const autoware_msgs::LaneArray::ConstPtr& lane_array,
  std::vector<autoware_msgs::LaneArray> *devided_lane_array)
{
  if (!devided_lane_array)
  {
    return;
  }
  ///////////////////////////////////////////for only 1 lane
  devided_lane_array->resize(1);
  devided_lane_array->at(0) = *lane_array;
  ///////////////////////////////////////////for only 1 lane
}

void WaypointManager::setPositionStop(autoware_msgs::LaneArray *lane_array)
{
  for (auto &el : lane_array->lanes)
  {
    unsigned int size = el.waypoints.size();
    geometry_msgs::Point &p0 = el.waypoints[size - 1].pose.pose.position;
    unsigned int valid_index = size;
    for (unsigned int i = 0; i < size; i++)
    {
      const int index = size - i - 1;
      geometry_msgs::Point &p1 = el.waypoints[index].pose.pose.position;
      double dist = std::hypot(p0.x - p1.x, p0.y - p1.y);
      if (dist >= pstop_distance_)
      {
        valid_index = index;
        break;
      }
    }
    if (valid_index == size)
    {
      continue;
    }
    el.waypoints[valid_index].wpstate.event_state = autoware_msgs::WaypointState::TYPE_EVENT_POSITION_STOP;
  }
}

void WaypointManager::replan(autoware_msgs::LaneArray* lane_array)
{
  if (!lane_array)
  {
    return;
  }
  for (auto &el : lane_array->lanes)
  {
    replanner_.replanLaneWaypointVel(&el);
  }
}

void WaypointManager::publishLaneArray()
{
  autoware_msgs::LaneArray array(devided_lane_array_[lane_idx_]);
  if (replanning_mode_)
  {
    replan(&array);
    setPositionStop(&array);
    lane_pub_.publish(array);
  }
  else
  {
    setPositionStop(&array);
    lane_pub_.publish(array);
  }
}

void WaypointManager::laneCallback(const autoware_msgs::LaneArray::ConstPtr& lane_array)
{
  devided_lane_array_.clear();
  lane_idx_ = 0;
  devideLane(lane_array, &devided_lane_array_);
  publishLaneArray();
}

void WaypointManager::stateCallback(const std_msgs::String::ConstPtr& state)
{
  std::vector<autoware_msgs::LaneArray> &dl = devided_lane_array_;
  if (dl.empty() || lane_idx_ >= dl.size() || state->data != "KTurn")
  {
    return;
  }
  ++lane_idx_;
  publishLaneArray();
}

void WaypointManager::configCallback(const autoware_msgs::ConfigWaypointReplanner::ConstPtr& conf)
{
  replanning_mode_ = conf->replanning_mode;
  replanner_.initParameter(conf);
  std::vector<autoware_msgs::LaneArray> &dl = devided_lane_array_;
  if (dl.empty() || lane_idx_ >= dl.size())
  {
    return;
  }
  publishLaneArray();
}

void WaypointManager::wfConfigCallback(const autoware_msgs::ConfigWaypointFollower::ConstPtr& conf)
{
  if (pstop_distance_ == conf->minimum_lookahead_distance)
  {
    return;
  }
  pstop_distance_ = conf->minimum_lookahead_distance;
  std::vector<autoware_msgs::LaneArray> &dl = devided_lane_array_;
  if (dl.empty() || lane_idx_ >= dl.size())
  {
    return;
  }
  publishLaneArray();
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_manager");
  waypoint_maker::WaypointManager wm;
  ros::spin();

  return 0;
}

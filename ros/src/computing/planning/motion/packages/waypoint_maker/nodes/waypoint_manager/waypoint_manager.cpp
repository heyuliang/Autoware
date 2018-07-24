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

WaypointManager::WaypointManager() : lane_idx_(0)
{
  lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>("/based/lane_waypoints_array", 10, true);
  lane_sub_ = nh_.subscribe("/based/lane_waypoints_raw", 1, &WaypointManager::laneCallback, this);
  state_sub_ = nh_.subscribe("/decision_maker/state", 1, &WaypointManager::stateCallback, this);
}

WaypointManager::~WaypointManager()
{
}

bool WaypointManager::devideLane(const autoware_msgs::LaneArray::ConstPtr& lane_array,
  std::vector<autoware_msgs::LaneArray> *devided_lane_array)
{
  if (!devided_lane_array)
  {
    return false;
  }
  ///////////////////////////////////////////for only 1 lane
  devided_lane_array->resize(1);
  devided_lane_array->at(0) = *lane_array;
  ///////////////////////////////////////////for only 1 lane
  return true;
}

void WaypointManager::laneCallback(const autoware_msgs::LaneArray::ConstPtr& lane_array)
{
  devided_lane_array_.clear();
  lane_idx_ = 0;
  devideLane(lane_array, &devided_lane_array_);
  lane_pub_.publish(devided_lane_array_[lane_idx_]);
}

void WaypointManager::stateCallback(const std_msgs::String::ConstPtr& state)
{
  if (devided_lane_array_.empty() || lane_idx_ >= devided_lane_array_.size())
  {
    return;
  }
  else if (state->data != "KTurn")
  {
    return;
  }
  lane_pub_.publish(devided_lane_array_[++lane_idx_]);
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_manager");
  waypoint_maker::WaypointManager wm;
  ros::spin();

  return 0;
}

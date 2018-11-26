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

#ifndef AUTOWARE_MAP_AUTOWARE_MAP_H
#define AUTOWARE_MAP_AUTOWARE_MAP_H

#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>

#include <autoware_map_msgs/LaneArray.h>
#include <autoware_map_msgs/LaneAttrRelationArray.h>
#include <autoware_map_msgs/LaneRelationArray.h>
#include <autoware_map_msgs/LaneSignalLightRelationArray.h>
#include <autoware_map_msgs/LaneChangeRelationArray.h>
#include <autoware_map_msgs/OppositeLaneRelationArray.h>
#include <autoware_map_msgs/RouteArray.h>
#include <autoware_map_msgs/PointArray.h>
#include <autoware_map_msgs/AreaArray.h>
#include <autoware_map_msgs/SignalArray.h>
#include <autoware_map_msgs/SignalLightArray.h>
#include <autoware_map_msgs/WayareaArray.h>
#include <autoware_map_msgs/WaypointArray.h>
#include <autoware_map_msgs/WaypointLaneRelationArray.h>
#include <autoware_map_msgs/WaypointRelationArray.h>
#include <autoware_map_msgs/WaypointSignalRelationArray.h>

namespace autoware_map
{
using autoware_map_msgs::Lane;
using autoware_map_msgs::LaneArray;
using autoware_map_msgs::LaneAttrRelation;
using autoware_map_msgs::LaneAttrRelationArray;
using autoware_map_msgs::LaneRelation;
using autoware_map_msgs::LaneRelationArray;
using autoware_map_msgs::LaneSignalLightRelation;
using autoware_map_msgs::LaneSignalLightRelationArray;
using autoware_map_msgs::LaneChangeRelation;
using autoware_map_msgs::LaneChangeRelationArray;
using autoware_map_msgs::OppositeLaneRelation;
using autoware_map_msgs::OppositeLaneRelationArray;
using autoware_map_msgs::Point;
using autoware_map_msgs::PointArray;
using autoware_map_msgs::Area;
using autoware_map_msgs::AreaArray;
using autoware_map_msgs::Route;
using autoware_map_msgs::RouteArray;
using autoware_map_msgs::Signal;
using autoware_map_msgs::SignalArray;
using autoware_map_msgs::SignalLight;
using autoware_map_msgs::SignalLightArray;
using autoware_map_msgs::Wayarea;
using autoware_map_msgs::WayareaArray;
using autoware_map_msgs::Waypoint;
using autoware_map_msgs::WaypointArray;
using autoware_map_msgs::WaypointRelation;
using autoware_map_msgs::WaypointRelationArray;
using autoware_map_msgs::WaypointLaneRelation;
using autoware_map_msgs::WaypointLaneRelationArray;
using autoware_map_msgs::WaypointSignalRelation;
using autoware_map_msgs::WaypointSignalRelationArray;



using category_t = unsigned long long;
// using vector_map::Filter;
// using vector_map::Callback;

enum Category : category_t
{
    NONE = 0LLU,

    // Graphical Primitive Class
    LANE = 1LLU << 0,
    LANE_ATTR_RELATION = 1LLU << 1,
    LANE_RELATION = 1LLU << 2,
    LANE_SIGNAL_LIGHT_RELATION = 1LLU << 3,
    LANE_CHANGE_RELATION = 1LLU << 4,
    OPPOSITE_LANE_RELATION = 1LLU << 5,
    POINT = 1LLU << 6,
    AREA = 1LLU << 7,
    ROUTE = 1LLU << 8,
    SIGNAL = 1LLU << 9,
    SIGNAL_LIGHT = 1LLU << 10,
    WAYAREA = 1LLU << 11,
    WAYPOINT = 1LLU << 12,
    WAYPOINT_LANE_RELATION = 1LLU << 13,
    WAYPOINT_RELATION = 1LLU << 14,
    WAYPOINT_SIGNAL_RELATION = 1LLU << 15,
    ALL = (1LLU << 32) - 1
};

template <class T>
class Key
{
private:
  int id_;

public:
  Key()
  {
  }

  explicit Key(int id)
    : id_(id)
  {
  }

  void setId(int id)
  {
    id_ = id;
  }

  int getId() const
  {
    return id_;
  }

  bool operator<(const Key<T>& right) const
  {
    return id_ < right.getId();
  }
};

template <class T, class U>
using Updater = std::function<void(std::map<Key<T>, T>&, const U&)>;

template <class T>
using Callback = std::function<void(const T&)>;

template <class T>
using Filter = std::function<bool(const T&)>;

template <class T, class U>
class Handle
{
private:
  ros::Subscriber sub_;
  Updater<T, U> update_;
  std::vector<Callback<U>> cbs_;
  std::map<Key<T>, T> map_;

  void subscribe(const U& msg)
  {
    update_(map_, msg);
    for (const auto& cb : cbs_)
      cb(msg);
  }

public:
  Handle()
  {
  }

  void registerSubscriber(ros::NodeHandle& nh, const std::string& topic_name)
  {
    if(topic_name != std::string(sub_.getTopic()) )
    {
        sub_ = nh.subscribe(topic_name, 1, &Handle<T, U>::subscribe, this);
    }
  }

  void registerUpdater(const Updater<T, U>& update)
  {
    update_ = update;
  }

  void registerCallback(const Callback<U>& cb)
  {
    cbs_.push_back(cb);
  }

  T findByKey(const Key<T>& key) const
  {
    auto it = map_.find(key);
    if (it == map_.end())
      return T();
    return it->second;
  }

  std::vector<T> findByFilter(const Filter<T>& filter) const
  {
    std::vector<T> vector;
    for (const auto& pair : map_)
    {
      if (filter(pair.second))
        vector.push_back(pair.second);
    }
    return vector;
  }

  bool empty() const
  {
    return map_.empty();
  }
};


template <class T>
std::vector<T> parse(const std::string& csv_file)
{
    std::ifstream ifs(csv_file.c_str());
    std::string line;
    std::getline(ifs, line); // remove first line
    std::vector<T> objs;
    while (std::getline(ifs, line))
    {
        T obj;
        std::istringstream iss(line);
        iss >> obj;
        objs.push_back(obj);
    }
    return objs;
}

class AutowareMap
{

private:
    autoware_map::Handle<Lane, LaneArray> lane_;
    autoware_map::Handle<LaneAttrRelation, LaneAttrRelationArray> lane_attr_relation_;
    autoware_map::Handle<LaneRelation, LaneRelationArray> lane_relation_;
    autoware_map::Handle<LaneSignalLightRelation, LaneSignalLightRelationArray> lane_signal_light_relation_;
    autoware_map::Handle<LaneChangeRelation, LaneChangeRelationArray> lane_change_relation_;
    autoware_map::Handle<OppositeLaneRelation, OppositeLaneRelationArray> opposite_lane_relation_;
    autoware_map::Handle<Point, PointArray> point_;
    autoware_map::Handle<Area, AreaArray> area_;
    autoware_map::Handle<Route, RouteArray> route_;
    autoware_map::Handle<Signal, SignalArray> signal_;
    autoware_map::Handle<SignalLight, SignalLightArray> signal_light_;
    autoware_map::Handle<Wayarea, WayareaArray> wayarea_;
    autoware_map::Handle<Waypoint, WaypointArray> waypoint_;
    autoware_map::Handle<WaypointLaneRelation, WaypointLaneRelationArray> waypoint_lane_relation_;
    autoware_map::Handle<WaypointRelation, WaypointRelationArray> waypoint_relation_;
    autoware_map::Handle<WaypointSignalRelation, WaypointSignalRelationArray> waypoint_signal_relation_;


public:
    AutowareMap();
    void registerSubscriber(ros::NodeHandle& nh, category_t category);
    void subscribe(ros::NodeHandle& nh, category_t category);
    void subscribe(ros::NodeHandle& nh, category_t category, const ros::Duration& timeout);
    void subscribe(ros::NodeHandle& nh, category_t category, const size_t max_retries);

    Lane findByKey(const autoware_map::Key<Lane>& key) const;
    LaneAttrRelation findByKey(const autoware_map::Key<LaneAttrRelation>& key) const;
    LaneRelation findByKey(const autoware_map::Key<LaneRelation>& key) const;
    LaneSignalLightRelation findByKey(const autoware_map::Key<LaneSignalLightRelation>& key) const;
    LaneChangeRelation findByKey(const autoware_map::Key<LaneChangeRelation>& key) const;
    OppositeLaneRelation findByKey(const autoware_map::Key<OppositeLaneRelation>& key) const;
    Point findByKey(const autoware_map::Key<Point>& key) const;
    Area findByKey(const autoware_map::Key<Area>& key) const;
    Route findByKey(const autoware_map::Key<Route>& key) const;
    Signal findByKey(const autoware_map::Key<Signal>& key) const;
    SignalLight findByKey(const autoware_map::Key<SignalLight>& key) const;
    Wayarea findByKey(const autoware_map::Key<Wayarea>& key) const;
    Waypoint findByKey(const autoware_map::Key<Waypoint>& key) const;
    WaypointLaneRelation findByKey(const autoware_map::Key<WaypointLaneRelation>& key) const;
    WaypointRelation findByKey(const autoware_map::Key<WaypointRelation>& key) const;
    WaypointSignalRelation findByKey(const autoware_map::Key<WaypointSignalRelation>& key) const;

    std::vector<Lane> findByFilter(const Filter<Lane>& filter) const;
    std::vector<LaneAttrRelation> findByFilter(const Filter<LaneAttrRelation>& filter) const;
    std::vector<LaneRelation> findByFilter(const Filter<LaneRelation>& filter) const;
    std::vector<LaneSignalLightRelation> findByFilter(const Filter<LaneSignalLightRelation>& filter) const;
    std::vector<LaneChangeRelation> findByFilter(const Filter<LaneChangeRelation>& filter) const;
    std::vector<OppositeLaneRelation> findByFilter(const Filter<OppositeLaneRelation>& filter) const;
    std::vector<Point> findByFilter(const Filter<Point>& filter) const;
    std::vector<Area> findByFilter(const Filter<Area>& filter) const;
    std::vector<Route> findByFilter(const Filter<Route>& filter) const;
    std::vector<Signal> findByFilter(const Filter<Signal>& filter) const;
    std::vector<SignalLight> findByFilter(const Filter<SignalLight>& filter) const;
    std::vector<Wayarea> findByFilter(const Filter<Wayarea>& filter) const;
    std::vector<Waypoint> findByFilter(const Filter<Waypoint>& filter) const;
    std::vector<WaypointLaneRelation> findByFilter(const Filter<WaypointLaneRelation>& filter) const;
    std::vector<WaypointRelation> findByFilter(const Filter<WaypointRelation>& filter) const;
    std::vector<WaypointSignalRelation> findByFilter(const Filter<WaypointSignalRelation>& filter) const;

    bool hasSubscribed(category_t category) const;
    category_t hasSubscribed() const;

    void registerCallback(const Callback<Lane>& cb);
    void registerCallback(const Callback<LaneAttrRelation>& cb);
    void registerCallback(const Callback<LaneRelation>& cb);
    void registerCallback(const Callback<LaneSignalLightRelation>& cb);
    void registerCallback(const Callback<LaneChangeRelation>& cb);
    void registerCallback(const Callback<OppositeLaneRelation>& cb);
    void registerCallback(const Callback<Point>& cb);
    void registerCallback(const Callback<Area>& cb);
    void registerCallback(const Callback<Route>& cb);
    void registerCallback(const Callback<Signal>& cb);
    void registerCallback(const Callback<SignalLight>& cb);
    void registerCallback(const Callback<Wayarea>& cb);
    void registerCallback(const Callback<Waypoint>& cb);
    void registerCallback(const Callback<WaypointLaneRelation>& cb);
    void registerCallback(const Callback<WaypointRelation>& cb);
    void registerCallback(const Callback<WaypointSignalRelation>& cb);

};



} //namespace

//
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Lane& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneAttrRelation& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneRelation& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneSignalLightRelation& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneChangeRelation& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::OppositeLaneRelation& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Point& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Area& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Route& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Signal& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::SignalLight& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Wayarea& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Waypoint& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::WaypointLaneRelation& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::WaypointRelation& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::WaypointSignalRelation& obj);
//
std::istream& operator>>(std::istream& os, autoware_map_msgs::Lane& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::LaneAttrRelation& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::LaneRelation& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::LaneSignalLightRelation& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::LaneChangeRelation& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::OppositeLaneRelation& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::Point& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::Area& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::Route& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::Signal& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::SignalLight& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::Wayarea& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::Waypoint& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::WaypointLaneRelation& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::WaypointSignalRelation& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::WaypointRelation& obj);

#endif // AUTOWARE_MAP_AUTOWARE_MAP_H

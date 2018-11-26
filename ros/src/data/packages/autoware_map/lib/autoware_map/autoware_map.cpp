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

#include <tf/transform_datatypes.h>
#include <autoware_map/autoware_map.h>
//
namespace autoware_map
{
namespace
{
void updateLane(std::map<autoware_map::Key<Lane>, Lane>& map, const LaneArray& msg)
{
    map = std::map<autoware_map::Key<Lane>, Lane>();
    for (const auto& item : msg.data)
    {
        if (item.lane_id == 0)
            continue;
        map.insert(std::make_pair(autoware_map::Key<Lane>(item.lane_id), item));
    }
}
void updateLaneAttrRelation(std::map<autoware_map::Key<LaneAttrRelation>, LaneAttrRelation>& map, const LaneAttrRelationArray& msg)
{
    //use this id for key since there are no unique id for relations
    int id = 1;
    map = std::map<autoware_map::Key<LaneAttrRelation>, LaneAttrRelation>();
    for (const auto& item : msg.data)
    {
        if (item.lane_id == 0)
            continue;
        map.insert(std::make_pair(autoware_map::Key<LaneAttrRelation>(id++), item));
    }
}
void updateLaneRelation(std::map<autoware_map::Key<LaneRelation>, LaneRelation>& map, const LaneRelationArray& msg)
{
    //use this id for key since there are no unique id for relations
    int id = 1;
    map = std::map<autoware_map::Key<LaneRelation>, LaneRelation>();
    for (const auto& item : msg.data)
    {
        if (item.lane_id == 0)
            continue;
        map.insert(std::make_pair(autoware_map::Key<LaneRelation>(id++), item));
    }
}
void updateLaneSignalLightRelation(std::map<autoware_map::Key<LaneSignalLightRelation>, LaneSignalLightRelation>& map, const LaneSignalLightRelationArray& msg)
{
    //use this id for key since there are no unique id for relations
    int id = 1;
    map = std::map<autoware_map::Key<LaneSignalLightRelation>, LaneSignalLightRelation>();
    for (const auto& item : msg.data)
    {
        if (item.lane_id == 0)
            continue;
        map.insert(std::make_pair(autoware_map::Key<LaneSignalLightRelation>(id++), item));
    }
}
void updateLaneChangeRelation(std::map<autoware_map::Key<LaneChangeRelation>, LaneChangeRelation>& map, const LaneChangeRelationArray& msg)
{
    //use this id for key since there are no unique id for relations
    int id = 1;
    map = std::map<autoware_map::Key<LaneChangeRelation>, LaneChangeRelation>();
    for (const auto& item : msg.data)
    {
        if (item.lane_id == 0)
            continue;
        map.insert(std::make_pair(autoware_map::Key<LaneChangeRelation>(id++), item));
    }
}
void updateOppositeLaneRelation(std::map<autoware_map::Key<OppositeLaneRelation>, OppositeLaneRelation>& map, const OppositeLaneRelationArray& msg)
{
    //use this id for key since there are no unique id for relations
    int id = 1;
    map = std::map<autoware_map::Key<OppositeLaneRelation>, OppositeLaneRelation>();
    for (const auto& item : msg.data)
    {
        if (item.lane_id == 0)
            continue;
        map.insert(std::make_pair(autoware_map::Key<OppositeLaneRelation>(id++), item));
    }
}
void updatePoint(std::map<autoware_map::Key<Point>, Point>& map, const PointArray& msg)
{
    map = std::map<autoware_map::Key<Point>, Point>();
    for (const auto& item : msg.data)
    {
        if (item.point_id == 0)
            continue;
        map.insert(std::make_pair(autoware_map::Key<Point>(item.point_id), item));
    }
}
void updateArea(std::map<autoware_map::Key<Area>, Area>& map, const AreaArray& msg)
{
    map = std::map<autoware_map::Key<Area>, Area>();
    for (const auto& item : msg.data)
    {
        if (item.area_id == 0)
            continue;
        map.insert(std::make_pair(autoware_map::Key<Area>(item.area_id), item));
    }
}
void updateRoute(std::map<autoware_map::Key<Route>, Route>& map, const RouteArray& msg)
{
    map = std::map<autoware_map::Key<Route>, Route>();
    for (const auto& item : msg.data)
    {
        if (item.route_id == 0)
            continue;
        map.insert(std::make_pair(autoware_map::Key<Route>(item.route_id), item));
    }
}
void updateSignal(std::map<autoware_map::Key<Signal>, Signal>& map, const SignalArray& msg)
{
    map = std::map<autoware_map::Key<Signal>, Signal>();
    for (const auto& item : msg.data)
    {
        if (item.signal_id == 0)
            continue;
        map.insert(std::make_pair(autoware_map::Key<Signal>(item.signal_id), item));
    }
}
void updateSignalLight(std::map<autoware_map::Key<SignalLight>, SignalLight>& map, const SignalLightArray& msg)
{
    map = std::map<autoware_map::Key<SignalLight>, SignalLight>();
    for (const auto& item : msg.data)
    {
        if (item.signal_light_id == 0)
            continue;
        map.insert(std::make_pair(autoware_map::Key<SignalLight>(item.signal_light_id), item));
    }
}
void updateWayarea(std::map<autoware_map::Key<Wayarea>, Wayarea>& map, const WayareaArray& msg)
{
    map = std::map<autoware_map::Key<Wayarea>, Wayarea>();
    for (const auto& item : msg.data)
    {
        if (item.wayarea_id == 0)
            continue;
        map.insert(std::make_pair(autoware_map::Key<Wayarea>(item.wayarea_id), item));
    }
}
void updateWaypoint(std::map<autoware_map::Key<Waypoint>, Waypoint>& map, const WaypointArray& msg)
{
    map = std::map<autoware_map::Key<Waypoint>, Waypoint>();
    for (const auto& item : msg.data)
    {
        if (item.waypoint_id == 0)
            continue;
        map.insert(std::make_pair(autoware_map::Key<Waypoint>(item.waypoint_id), item));
    }
}
void updateWaypointRelation(std::map<autoware_map::Key<WaypointRelation>, WaypointRelation>& map, const WaypointRelationArray& msg)
{
    //use this id for key since there are no unique id for relations
    int id = 1;
    map = std::map<autoware_map::Key<WaypointRelation>, WaypointRelation>();
    for (const auto& item : msg.data)
    {
        if (item.waypoint_id == 0)
            continue;
        map.insert(std::make_pair(autoware_map::Key<WaypointRelation>(id++), item));
    }
}
void updateWaypointLaneRelation(std::map<autoware_map::Key<WaypointLaneRelation>, WaypointLaneRelation>& map, const WaypointLaneRelationArray& msg)
{
    //use this id for key since there are no unique id for relations
    int id = 1;
    map = std::map<autoware_map::Key<WaypointLaneRelation>, WaypointLaneRelation>();
    for (const auto& item : msg.data)
    {
        if (item.waypoint_id == 0)
            continue;
        map.insert(std::make_pair(autoware_map::Key<WaypointLaneRelation>(id++), item));
    }
}
void updateWaypointSignalRelation(std::map<autoware_map::Key<WaypointSignalRelation>, WaypointSignalRelation>& map, const WaypointSignalRelationArray& msg)
{
    //use this id for key since there are no unique id for relations
    int id = 1;
    map = std::map<autoware_map::Key<WaypointSignalRelation>, WaypointSignalRelation>();
    for (const auto& item : msg.data)
    {
        if (item.waypoint_id == 0)
            continue;
        map.insert(std::make_pair(autoware_map::Key<WaypointSignalRelation>(id++), item));
    }
}
}   // namespace
bool AutowareMap::hasSubscribed(category_t category) const
{
    if (category & LANE)
    {
        if (lane_.empty())
            return false;
    }
    if (category & LANE_ATTR_RELATION)
    {
        if (lane_attr_relation_.empty())
            return false;
    }
    if (category & LANE_RELATION)
    {
        if (lane_relation_.empty())
            return false;
    }
    if (category & LANE_SIGNAL_LIGHT_RELATION)
    {
        if (lane_signal_light_relation_.empty())
            return false;
    }
    if (category & LANE_CHANGE_RELATION)
    {
        if (lane_change_relation_.empty())
            return false;
    }
    if (category & OPPOSITE_LANE_RELATION)
    {
        if (opposite_lane_relation_.empty())
            return false;
    }
    if (category & POINT)
    {
        if (point_.empty())
            return false;
    }
    if (category & AREA)
    {
        if (area_.empty())
            return false;
    }
    if (category & ROUTE)
    {
        if (route_.empty())
            return false;
    }
    if (category & SIGNAL)
    {
        if (signal_.empty())
            return false;
    }
    if (category & SIGNAL_LIGHT)
    {
        if (signal_light_.empty())
            return false;
    }
    if (category & WAYAREA)
    {
        if (wayarea_.empty())
            return false;
    }
    if (category & WAYPOINT)
    {
        if (waypoint_.empty())
            return false;
    }
    if (category & WAYPOINT_LANE_RELATION)
    {
        if (waypoint_lane_relation_.empty())
            return false;
    }
    if (category & WAYPOINT_RELATION)
    {
        if (waypoint_relation_.empty())
            return false;
    }
    if (category & WAYPOINT_SIGNAL_RELATION)
    {
        if (waypoint_signal_relation_.empty())
            return false;
    }
    return true;
}
category_t AutowareMap::hasSubscribed() const
{
    category_t category=NONE;
    if (!lane_.empty())
        category |= LANE;
    if (!lane_attr_relation_.empty())
        category |= LANE_ATTR_RELATION;
    if (!lane_relation_.empty())
        category |= LANE_RELATION;
    if (!lane_signal_light_relation_.empty())
        category |= LANE_SIGNAL_LIGHT_RELATION;
    if (!lane_change_relation_.empty())
        category |= LANE_CHANGE_RELATION;
    if (!opposite_lane_relation_.empty())
        category |= OPPOSITE_LANE_RELATION;
    if (!point_.empty())
        category |= POINT;
    if (!area_.empty())
        category |= AREA;
    if (!route_.empty())
        category |= ROUTE;
    if (!signal_.empty())
        category |= SIGNAL;
    if (!signal_light_.empty())
        category |= SIGNAL_LIGHT;
    if (!wayarea_.empty())
        category |= WAYAREA;
    if (!waypoint_.empty())
        category |= WAYPOINT;
    if (!waypoint_lane_relation_.empty())
        category |= WAYPOINT_LANE_RELATION;
    if (!waypoint_relation_.empty())
        category |= WAYPOINT_RELATION;
    if (!waypoint_signal_relation_.empty())
        category |= WAYPOINT_SIGNAL_RELATION;

    return category;
}

void AutowareMap::registerSubscriber(ros::NodeHandle& nh, category_t category)
{
    if (category & LANE)
    {
        lane_.registerSubscriber(nh, "/autoware_map_info/lane");
        lane_.registerUpdater(updateLane);
    }
    if (category & LANE_ATTR_RELATION)
    {
        lane_attr_relation_.registerSubscriber(nh, "/autoware_map_info/lane_attr_relation");
        lane_attr_relation_.registerUpdater(updateLaneAttrRelation);
    }
    if (category & LANE_RELATION)
    {
        lane_relation_.registerSubscriber(nh, "/autoware_map_info/lane_relation");
        lane_relation_.registerUpdater(updateLaneRelation);
    }
    if (category & LANE_SIGNAL_LIGHT_RELATION)
    {
        lane_signal_light_relation_.registerSubscriber(nh, "/autoware_map_info/lane_signal_light_relation");
        lane_signal_light_relation_.registerUpdater(updateLaneSignalLightRelation);
    }
    if (category & LANE_CHANGE_RELATION)
    {
        lane_change_relation_.registerSubscriber(nh, "/autoware_map_info/lane_change_relation");
        lane_change_relation_.registerUpdater(updateLaneChangeRelation);
    }
    if (category & OPPOSITE_LANE_RELATION)
    {
        opposite_lane_relation_.registerSubscriber(nh, "/autoware_map_info/opposite_lane_relation");
        opposite_lane_relation_.registerUpdater(updateOppositeLaneRelation);
    }
    if (category & POINT)
    {
        point_.registerSubscriber(nh, "/autoware_map_info/point");
        point_.registerUpdater(updatePoint);
    }
    if (category & AREA)
    {
        area_.registerSubscriber(nh, "/autoware_map_info/area");
        area_.registerUpdater(updateArea);
    }
    if (category & ROUTE)
    {
        route_.registerSubscriber(nh, "/autoware_map_info/route");
        route_.registerUpdater(updateRoute);
    }
    if (category & SIGNAL)
    {
        signal_.registerSubscriber(nh, "/autoware_map_info/signal");
        signal_.registerUpdater(updateSignal);
    }
    if (category & SIGNAL_LIGHT)
    {
        signal_light_.registerSubscriber(nh, "/autoware_map_info/signal_light");
        signal_light_.registerUpdater(updateSignalLight);
    }
    if (category & WAYAREA)
    {
        wayarea_.registerSubscriber(nh, "/autoware_map_info/wayarea");
        wayarea_.registerUpdater(updateWayarea);
    }
    if (category & WAYPOINT)
    {
        waypoint_.registerSubscriber(nh, "/autoware_map_info/waypoint");
        waypoint_.registerUpdater(updateWaypoint);
    }
    if (category & WAYPOINT_LANE_RELATION)
    {
        waypoint_lane_relation_.registerSubscriber(nh, "/autoware_map_info/waypoint_lane_relation");
        waypoint_lane_relation_.registerUpdater(updateWaypointLaneRelation);
    }
    if (category & WAYPOINT_RELATION)
    {
        waypoint_relation_.registerSubscriber(nh, "/autoware_map_info/waypoint_relation");
        waypoint_relation_.registerUpdater(updateWaypointRelation);
    }
    if (category & WAYPOINT_SIGNAL_RELATION)
    {
        waypoint_signal_relation_.registerSubscriber(nh, "/autoware_map_info/waypoint_signal_relation");
        waypoint_signal_relation_.registerUpdater(updateWaypointSignalRelation);
    }

}

AutowareMap::AutowareMap()
{
}


void AutowareMap::subscribe(ros::NodeHandle& nh, category_t category)
{
    registerSubscriber(nh, category);
    ros::Rate rate(10);
    while (ros::ok() && !hasSubscribed(category))
    {
        ros::spinOnce();
        rate.sleep();
    }
}

void AutowareMap::subscribe(ros::NodeHandle& nh, category_t category, const ros::Duration& timeout)
{
    registerSubscriber(nh, category);
    ros::Rate rate(10);
    ros::Time end = ros::Time::now() + timeout;
    while (ros::ok() && !hasSubscribed(category) && ros::Time::now() < end)
    {
        ros::spinOnce();
        rate.sleep();
    }
}

void AutowareMap::subscribe(ros::NodeHandle& nh, category_t category, const size_t max_retries)
{
    size_t tries = 0;
    registerSubscriber(nh, category);
    ros::Rate rate(10);
    while (ros::ok() && !hasSubscribed(category) && tries++ < max_retries)
    {
        ros::spinOnce();
        rate.sleep();
    }
}


Lane AutowareMap::findByKey(const autoware_map::Key<Lane>& key) const
{
    return lane_.findByKey(key);
}
LaneAttrRelation AutowareMap::findByKey(const autoware_map::Key<LaneAttrRelation>& key) const
{
    return lane_attr_relation_.findByKey(key);
}
LaneRelation AutowareMap::findByKey(const autoware_map::Key<LaneRelation>& key) const
{
    return lane_relation_.findByKey(key);
}
LaneSignalLightRelation AutowareMap::findByKey(const autoware_map::Key<LaneSignalLightRelation>& key) const
{
    return lane_signal_light_relation_.findByKey(key);
}
LaneChangeRelation AutowareMap::findByKey(const autoware_map::Key<LaneChangeRelation>& key) const
{
    return lane_change_relation_.findByKey(key);
}
OppositeLaneRelation AutowareMap::findByKey(const autoware_map::Key<OppositeLaneRelation>& key) const
{
    return opposite_lane_relation_.findByKey(key);
}
Point AutowareMap::findByKey(const autoware_map::Key<Point>& key) const
{
    return point_.findByKey(key);
}
Area AutowareMap::findByKey(const autoware_map::Key<Area>& key) const
{
    return area_.findByKey(key);
}
Route AutowareMap::findByKey(const autoware_map::Key<Route>& key) const
{
    return route_.findByKey(key);
}
Signal AutowareMap::findByKey(const autoware_map::Key<Signal>& key) const
{
    return signal_.findByKey(key);
}
SignalLight AutowareMap::findByKey(const autoware_map::Key<SignalLight>& key) const
{
    return signal_light_.findByKey(key);
}
Wayarea AutowareMap::findByKey(const autoware_map::Key<Wayarea>& key) const
{
    return wayarea_.findByKey(key);
}
Waypoint AutowareMap::findByKey(const autoware_map::Key<Waypoint>& key) const
{
    return waypoint_.findByKey(key);
}
WaypointLaneRelation AutowareMap::findByKey(const autoware_map::Key<WaypointLaneRelation>& key) const
{
    return waypoint_lane_relation_.findByKey(key);
}
WaypointRelation AutowareMap::findByKey(const autoware_map::Key<WaypointRelation>& key) const
{
    return waypoint_relation_.findByKey(key);
}
WaypointSignalRelation AutowareMap::findByKey(const autoware_map::Key<WaypointSignalRelation>& key) const
{
    return waypoint_signal_relation_.findByKey(key);
}
std::vector<Lane> AutowareMap::findByFilter(const Filter<Lane>& filter) const
{
    return lane_.findByFilter(filter);
}
std::vector<LaneAttrRelation> AutowareMap::findByFilter(const Filter<LaneAttrRelation>& filter) const
{
    return lane_attr_relation_.findByFilter(filter);
}
std::vector<LaneRelation> AutowareMap::findByFilter(const Filter<LaneRelation>& filter) const
{
    return lane_relation_.findByFilter(filter);
}
std::vector<LaneSignalLightRelation> AutowareMap::findByFilter(const Filter<LaneSignalLightRelation>& filter) const
{
    return lane_signal_light_relation_.findByFilter(filter);
}
std::vector<LaneChangeRelation> AutowareMap::findByFilter(const Filter<LaneChangeRelation>& filter) const
{
    return lane_change_relation_.findByFilter(filter);
}
std::vector<OppositeLaneRelation> AutowareMap::findByFilter(const Filter<OppositeLaneRelation>& filter) const
{
    return opposite_lane_relation_.findByFilter(filter);
}
std::vector<Point> AutowareMap::findByFilter(const Filter<Point>& filter) const
{
    return point_.findByFilter(filter);
}
std::vector<Area> AutowareMap::findByFilter(const Filter<Area>& filter) const
{
    return area_.findByFilter(filter);
}
std::vector<Route> AutowareMap::findByFilter(const Filter<Route>& filter) const
{
    return route_.findByFilter(filter);
}
std::vector<Signal> AutowareMap::findByFilter(const Filter<Signal>& filter) const
{
    return signal_.findByFilter(filter);
}
std::vector<SignalLight> AutowareMap::findByFilter(const Filter<SignalLight>& filter) const
{
    return signal_light_.findByFilter(filter);
}
std::vector<Wayarea> AutowareMap::findByFilter(const Filter<Wayarea>& filter) const
{
    return wayarea_.findByFilter(filter);
}
std::vector<Waypoint> AutowareMap::findByFilter(const Filter<Waypoint>& filter) const
{
    return waypoint_.findByFilter(filter);
}
std::vector<WaypointLaneRelation> AutowareMap::findByFilter(const Filter<WaypointLaneRelation>& filter) const
{
    return waypoint_lane_relation_.findByFilter(filter);
}
std::vector<WaypointRelation> AutowareMap::findByFilter(const Filter<WaypointRelation>& filter) const
{
    return waypoint_relation_.findByFilter(filter);
}
std::vector<WaypointSignalRelation> AutowareMap::findByFilter(const Filter<WaypointSignalRelation>& filter) const
{
    return waypoint_signal_relation_.findByFilter(filter);
}


}

std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Lane& obj)
{
    os << obj.lane_id << ","
    << obj.start_waypoint_id << ","
    << obj.end_waypoint_id << ","
    << obj.lane_number << ","
    << obj.num_of_lanes << ","
    << obj.speed_limit << ","
    << obj.length << ","
    << obj.width_limit << ","
    << obj.height_limit << ","
    << obj.weight_limit;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneAttrRelation& obj)
{
    os << obj.lane_id << ","
    << obj.attribute_type << ","
    << obj.area_id;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneRelation& obj)
{
    os << obj.lane_id << ","
    << obj.next_lane_id << ","
    << obj.blinker;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneSignalLightRelation& obj)
{
    os << obj.lane_id << ","
    << obj.signal_light_id;
    return os;
}

std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneChangeRelation& obj)
{
    os << obj.lane_id << ","
    << obj.next_lane_id << ","
    << obj.blinker;
    return os;
}

std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::OppositeLaneRelation& obj)
{
    os << obj.lane_id << ","
    << obj.opposite_lane_id;
    return os;
}

std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Point& obj)
{
    os << obj.point_id << ","
    << obj.x << ","
    << obj.y << ","
    << obj.z << ","
    << obj.epsg << ","
    << obj.pcd << ","
    << obj.lat << ","
    << obj.lng;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Area& obj)
{
    os << obj.area_id << ",";
    for ( auto id = obj.point_ids.begin(); id != obj.point_ids.end(); id++)
    {
        os << *id;
        if( id+1 != obj.point_ids.end() )
            os << ":";
    }
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Route& obj)
{
    os << obj.route_id << ","
    << obj.start_waypoint_id << ","
    << obj.end_waypoint_id << ","
    << obj.begin_lane_id << ","
    << obj.finish_lane_id << ","
    << obj.min_lane_width << ","
    << obj.max_lane_width << ","
    << obj.length << ","
    << obj.max_weight;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Signal& obj)
{
    os << obj.signal_id;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::SignalLight& obj)
{
    os << obj.signal_light_id << ","
    << obj.signal_id << ","
    << obj.point_id << ","
    << obj.horizontal_angle << ","
    << obj.vertical_angle << ","
    << obj.color_type << ","
    << obj.arrow_type;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Wayarea& obj)
{
    os << obj.wayarea_id << ","
    << obj.area_id;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Waypoint& obj)
{
    os << obj.waypoint_id << ","
    << obj.point_id << ","
    << obj.yaw << ","
    << obj.velocity << ","
    << obj.stop_line << ","
    << obj.width << ","
    << obj.height;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::WaypointLaneRelation& obj)
{
    os << obj.waypoint_id << ","
    << obj.lane_id;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::WaypointRelation& obj)
{
    os << obj.waypoint_id << ","
    << obj.next_waypoint_id << ","
    << obj.blinker << ","
    << obj.distance;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::WaypointSignalRelation& obj)
{
    os << obj.waypoint_id << ","
    << obj.signal_id;
    return os;
}

std::istream& operator>>(std::istream& is, autoware_map_msgs::Lane& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns[0]);
    obj.start_waypoint_id = std::stoi(columns[1]);
    obj.end_waypoint_id = std::stoi(columns[2]);
    obj.lane_number = std::stoi(columns[3]);
    obj.num_of_lanes = std::stoi(columns[4]);
    obj.speed_limit = std::stod(columns[5]);
    obj.length = std::stod(columns[6]);
    obj.width_limit = std::stod(columns[7]);
    obj.height_limit = std::stod(columns[8]);
    obj.weight_limit = std::stod(columns[9]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::LaneAttrRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns[0]);
    obj.attribute_type = std::stoi(columns[1]);
    obj.area_id = std::stoi(columns[2]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::LaneRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns[0]);
    obj.next_lane_id = std::stoi(columns[1]);
    obj.blinker = std::stoi(columns[2]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::LaneSignalLightRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns[0]);
    obj.signal_light_id = std::stoi(columns[1]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::LaneChangeRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns[0]);
    obj.next_lane_id = std::stoi(columns[1]);
    obj.blinker = std::stoi(columns[2]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::OppositeLaneRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns[0]);
    obj.opposite_lane_id = std::stoi(columns[1]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::Point& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.point_id = std::stoi(columns[0]);
    obj.x = std::stod(columns[1]);
    obj.y = std::stod(columns[2]);
    obj.z = std::stod(columns[3]);
    obj.epsg = std::stoi(columns[4]);
    obj.pcd = columns[5];
    obj.lat = std::stod(columns[6]);
    obj.lng = std::stod(columns[7]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::Area& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.area_id = std::stoi(columns[0]);
    std::stringstream ss(columns[1]);
    while (std::getline(ss, column, ':' )) {
        obj.point_ids.push_back( std::stoi(column) );
    }

    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::Route& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.route_id = std::stoi(columns[0]);
    obj.start_waypoint_id = std::stoi(columns[1]);
    obj.end_waypoint_id = std::stoi(columns[2]);
    obj.begin_lane_id = std::stoi(columns[3]);
    obj.finish_lane_id = std::stoi(columns[4]);
    obj.min_lane_width = std::stod(columns[5]);
    obj.max_lane_width = std::stod(columns[6]);
    obj.length = std::stod(columns[7]);
    obj.max_weight = std::stod(columns[8]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::Signal& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.signal_id = std::stoi(columns[0]);
    // obj.signal_light_id = std::stoi(columns[1]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::SignalLight& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.signal_light_id = std::stoi(columns[0]);
    obj.signal_id = std::stoi(columns[1]);
    obj.point_id = std::stoi(columns[2]);
    obj.horizontal_angle = std::stod(columns[3]);
    obj.vertical_angle = std::stod(columns[4]);
    obj.color_type = std::stoi(columns[5]);
    obj.arrow_type = std::stoi(columns[6]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::Wayarea& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.wayarea_id = std::stoi(columns[0]);
    obj.area_id = std::stoi(columns[1]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::Waypoint& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.waypoint_id = std::stoi(columns[0]);
    obj.point_id = std::stoi(columns[1]);
    obj.yaw = std::stod(columns[2]);
    obj.velocity = std::stod(columns[3]);
    obj.stop_line = std::stoi(columns[4]);
    obj.width = std::stod(columns[5]);
    obj.height = std::stod(columns[6]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::WaypointLaneRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.waypoint_id = std::stoi(columns[0]);
    obj.lane_id = std::stoi(columns[1]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::WaypointRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.waypoint_id = std::stoi(columns[0]);
    obj.next_waypoint_id = std::stoi(columns[1]);
    obj.blinker = std::stoi(columns[2]);
    obj.distance = std::stod(columns[3]);
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::WaypointSignalRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.waypoint_id = std::stoi(columns[0]);
    obj.signal_id = std::stoi(columns[1]);
    return is;
}

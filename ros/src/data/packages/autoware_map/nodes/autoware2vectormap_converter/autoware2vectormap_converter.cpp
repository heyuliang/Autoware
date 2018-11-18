// #include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_map/autoware_map.h>
#include <vector_map/vector_map.h>
#include <sys/stat.h>

using vector_map::isValidMarker;
using vector_map::createVectorMarker;
using vector_map::createAreaMarker;

using vector_map::Color;
using vector_map::VectorMap;


void insertMarkerArray(visualization_msgs::MarkerArray& a1, const visualization_msgs::MarkerArray& a2)
{
    a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
}

template <class T, class U>
U createObjectArray(const std::vector<T> data)
{
    U obj_array;
    // NOTE: Autoware want to use map messages with or without /use_sim_time.
    // Therefore we don't set obj_array.header.stamp.
    // obj_array.header.stamp = ros::Time::now();
    obj_array.header.frame_id = "map";
    obj_array.data = data;
    // test();
    return obj_array;
}


visualization_msgs::Marker createLinkedLineMarker(const std::string& ns, int id, Color color, const VectorMap& vmap,
                                                  const vector_map_msgs::Line& line)
{
    vector_map_msgs::Area area;
    area.aid = 1; // must set valid aid
    area.slid = line.lid;
    return createAreaMarker(ns, id, color, vmap, area);
}

visualization_msgs::MarkerArray createStopLineMarkerArray(const VectorMap& vmap, Color color)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    for (const auto& stop_line : vmap.findByFilter([] (const vector_map_msgs::StopLine& stop_line){return true; }))
    {
        if (stop_line.lid == 0)
        {
            ROS_ERROR_STREAM("[createStopLineMarkerArray] invalid stop_line: " << stop_line);
            continue;
        }

        vector_map_msgs::Line line = vmap.findByKey(vector_map::Key<vector_map_msgs::Line>(stop_line.lid));
        if (line.lid == 0)
        {
            ROS_ERROR_STREAM("[createStopLineMarkerArray] invalid line: " << line);
            continue;
        }

        if (line.blid == 0) // if beginning line
        {
            visualization_msgs::Marker marker = createLinkedLineMarker("stop_line", id++, color, vmap, line);
            if (isValidMarker(marker))
                marker_array.markers.push_back(marker);
            else
                ROS_ERROR_STREAM("[createStopLineMarkerArray] failed createLinkedLineMarker: " << line);
        }
    }
    return marker_array;
}

visualization_msgs::MarkerArray createCrossWalkMarkerArray(const VectorMap& vmap, Color color)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    for (const auto& cross_walk : vmap.findByFilter([] (const vector_map_msgs::CrossWalk& cross_walk){return true; }))
    {
        if (cross_walk.aid == 0)
        {
            ROS_ERROR_STREAM("[createCrossWalkMarkerArray] invalid cross_walk: " << cross_walk);
            continue;
        }

        vector_map_msgs::Area area = vmap.findByKey(vector_map::Key<vector_map_msgs::Area>(cross_walk.aid));
        if (area.aid == 0)
        {
            ROS_ERROR_STREAM("[createCrossWalkMarkerArray] invalid area: " << area);
            continue;
        }

        visualization_msgs::Marker marker = createAreaMarker("cross_walk", id++, color, vmap, area);
        if (isValidMarker(marker))
            marker_array.markers.push_back(marker);
        else
            ROS_ERROR_STREAM("[createCrossWalkMarkerArray] failed createAreaMarker: " << area);
    }
    return marker_array;
}

visualization_msgs::MarkerArray createSignalMarkerArray(const VectorMap& vmap, Color red_color, Color blue_color,
                                                        Color yellow_color, Color other_color, Color pole_color)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    for (const auto& signal : vmap.findByFilter([] (const vector_map_msgs::Signal& signal){return true; }))
    {
        if (signal.vid == 0)
        {
            ROS_ERROR_STREAM("[createSignalMarkerArray] invalid signal: " << signal);
            continue;
        }

        vector_map_msgs::Vector vector = vmap.findByKey(vector_map::Key<vector_map_msgs::Vector>(signal.vid));
        if (vector.vid == 0)
        {
            ROS_ERROR_STREAM("[createSignalMarkerArray] invalid vector: " << vector);
            continue;
        }

        vector_map_msgs::Pole pole;
        if (signal.plid != 0)
        {
            pole = vmap.findByKey(vector_map::Key<vector_map_msgs::Pole>(signal.plid));
            if (pole.plid == 0)
            {
                ROS_ERROR_STREAM("[createSignalMarkerArray] invalid pole: " << pole);
                continue;
            }
        }

        visualization_msgs::Marker vector_marker;
        switch (signal.type)
        {
            case vector_map_msgs::Signal::RED:
            case vector_map_msgs::Signal::PEDESTRIAN_RED:
                vector_marker = createVectorMarker("signal", id++, red_color, vmap, vector);
                break;
            case vector_map_msgs::Signal::BLUE:
            case vector_map_msgs::Signal::PEDESTRIAN_BLUE:
                vector_marker = createVectorMarker("signal", id++, blue_color, vmap, vector);
                break;
            case vector_map_msgs::Signal::YELLOW:
                vector_marker = createVectorMarker("signal", id++, yellow_color, vmap, vector);
                break;
            case vector_map_msgs::Signal::RED_LEFT:
                vector_marker = createVectorMarker("signal", id++, Color::LIGHT_RED, vmap, vector);
                break;
            case vector_map_msgs::Signal::BLUE_LEFT:
                vector_marker = createVectorMarker("signal", id++, Color::LIGHT_GREEN, vmap, vector);
                break;
            case vector_map_msgs::Signal::YELLOW_LEFT:
                vector_marker = createVectorMarker("signal", id++, Color::LIGHT_YELLOW, vmap, vector);
                break;
            case vector_map_msgs::Signal::OTHER:
                vector_marker = createVectorMarker("signal", id++, other_color, vmap, vector);
                break;
            default:
                ROS_WARN_STREAM("[createSignalMarkerArray] unknown signal.type: " << signal.type << " Creating Marker as OTHER.");
                vector_marker = createVectorMarker("signal", id++, Color::GRAY, vmap, vector);
                break;
        }
        if (isValidMarker(vector_marker))
            marker_array.markers.push_back(vector_marker);
        else
            ROS_ERROR_STREAM("[createSignalMarkerArray] failed createVectorMarker: " << vector);

        // if (signal.plid != 0)
        // {
        //   visualization_msgs::Marker pole_marker = createPoleMarker("signal", id++, pole_color, vmap, pole);
        //   if (isValidMarker(pole_marker))
        //     marker_array.markers.push_back(pole_marker);
        //   else
        //     ROS_ERROR_STREAM("[createSignalMarkerArray] failed createPoleMarker: " << pole);
        // }
    }
    return marker_array;
}

visualization_msgs::MarkerArray createCrossRoadMarkerArray(const VectorMap& vmap, Color color)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    for (const auto& cross_road : vmap.findByFilter([] (const vector_map_msgs::CrossRoad& cross_road){return true; }))
    {
        if (cross_road.aid == 0)
        {
            ROS_ERROR_STREAM("[createCrossRoadMarkerArray] invalid cross_road: " << cross_road);
            continue;
        }

        vector_map_msgs::Area area = vmap.findByKey(vector_map::Key<vector_map_msgs::Area>(cross_road.aid));
        if (area.aid == 0)
        {
            ROS_ERROR_STREAM("[createCrossRoadMarkerArray] invalid area: " << area);
            continue;
        }

        visualization_msgs::Marker marker = createAreaMarker("cross_road", id++, color, vmap, area);
        if (isValidMarker(marker))
            marker_array.markers.push_back(marker);
        else
            ROS_ERROR_STREAM("[createCrossRoadMarkerArray] failed createAreaMarker: " << area);
    }
    return marker_array;
}
// visualization_msgs::MarkerArray createRailCrossingMarkerArray(const VectorMap& vmap, Color color)
// {
//   visualization_msgs::MarkerArray marker_array;
//   int id = 0;
//   for (const auto& rail_crossing : vmap.findByFilter([](const vector_map_msgs::RailCrossing& rail_crossing){return true;}))
//   {
//     if (rail_crossing.aid == 0)
//     {
//       ROS_ERROR_STREAM("[createRailCrossingMarkerArray] invalid rail_crossing: " << rail_crossing);
//       continue;
//     }
//
//     vector_map_msgs::Area area = vmap.findByKey(vector_map::Key<vector_map_msgs::Area>(rail_crossing.aid));
//     if (area.aid == 0)
//     {
//       ROS_ERROR_STREAM("[createRailCrossingMarkerArray] invalid area: " << area);
//       continue;
//     }
//
//     visualization_msgs::Marker marker = createAreaMarker("rail_crossing", id++, color, vmap, area);
//     if (isValidMarker(marker))
//       marker_array.markers.push_back(marker);
//     else
//       ROS_ERROR_STREAM("[createRailCrossingMarkerArray] failed createAreaMarker: " << area);
//   }
//   return marker_array;
// }
void createAreas(std::vector<autoware_map_msgs::Area> awm_areas, std::vector<vector_map_msgs::Area> &vmap_areas, std::vector<vector_map_msgs::Line> &vmap_lines)
{
    int line_id = vmap_lines.size() + 1;
    for ( auto awm_area : awm_areas)
    {
        vector_map_msgs::Area vmap_area;
        vmap_area.aid = awm_area.area_id;
        vmap_area.slid = line_id;

        //create lines that represent area
        auto end_itr = awm_area.point_ids.end();
        auto begin_itr = awm_area.point_ids.begin();
        for (auto point_itr = begin_itr; point_itr != end_itr; point_itr++)
        {
            vector_map_msgs::Line line;
            line.lid =line_id;
            line.bpid = *point_itr;

            if(point_itr == begin_itr)
            {
                line.blid = 0;
            }
            else{
                line.blid = line_id - 1;
            }

            if(point_itr + 1 == end_itr)
            {
                line.flid = 0;
                line.fpid = *begin_itr; //close the loop
            }
            else{
                line.flid = line_id + 1;
                line.fpid = *(point_itr + 1);
            }

            vmap_lines.push_back(line);
            vmap_area.elid = line_id;

            line_id++;
        }
        vmap_areas.push_back(vmap_area);
    }
}
//
void createCrossRoads(std::vector<autoware_map_msgs::LaneAttrRelation> awm_lane_attr_relations, std::vector<vector_map_msgs::CrossRoad> &vmap_cross_roads)
{
    unsigned int id = 1;
    for ( auto awm_relation : awm_lane_attr_relations)
    {
        if( awm_relation.attribute_type == autoware_map::LaneAttrRelation::INTERSECTION )
        {
            if(std::find_if(vmap_cross_roads.begin(), vmap_cross_roads.end(), [&](vector_map_msgs::CrossRoad cr){return cr.aid == awm_relation.area_id; }) != vmap_cross_roads.end())
            {
                continue;
            }
            vector_map_msgs::CrossRoad cross_road;
            cross_road.id = id++;
            cross_road.aid = awm_relation.area_id;
            cross_road.linkid = 0;

            vmap_cross_roads.push_back(cross_road);
        }
    }
}

void createCrossWalks(std::vector<autoware_map_msgs::LaneAttrRelation> awm_lane_attr_relations, std::vector<vector_map_msgs::CrossWalk> &vmap_cross_walks)
{
    unsigned int id = 1;
    for ( auto awm_relation : awm_lane_attr_relations)
    {
        if( awm_relation.attribute_type == autoware_map::LaneAttrRelation::CROSS_WALK )
        {
            if(std::find_if(vmap_cross_walks.begin(), vmap_cross_walks.end(), [&](vector_map_msgs::CrossWalk cw){return cw.aid == awm_relation.area_id; }) != vmap_cross_walks.end())
            {
                continue;
            }
            vector_map_msgs::CrossWalk cross_walk;
            cross_walk.id = id++;
            cross_walk.aid = awm_relation.area_id;
            cross_walk.linkid = 0;

            vmap_cross_walks.push_back(cross_walk);
        }
    }
}

void createPoints(std::vector<autoware_map_msgs::Point> awm_points, std::vector<vector_map_msgs::Point> &vmap_points)
{
    bool epsg_fail_flag = false;
    for ( auto awm_pt : awm_points)
    {
        vector_map_msgs::Point vmap_point;
        vmap_point.pid = awm_pt.point_id;
        vmap_point.b =  awm_pt.lat;
        vmap_point.l =  awm_pt.lng;
        vmap_point.h = awm_pt.z;
        vmap_point.bx = awm_pt.x;
        vmap_point.ly = awm_pt.y;

        // japanese plane rectangular CS number calculated from epsg values;
        if(awm_pt.epsg >= 2443 && awm_pt.epsg <= 2461)
        {
            vmap_point.ref = awm_pt.epsg - 2442;
        }
        else{
            epsg_fail_flag = true;
            vmap_point.ref = 0;
        }

        //cannot convert mcodes from autoware_map_format
        vmap_point.mcode1 = 0;
        vmap_point.mcode2 = 0;
        vmap_point.mcode3 = 0;
        vmap_points.push_back(vmap_point);
    }
    if(epsg_fail_flag)
    {
        ROS_WARN_STREAM("no corresponding Japanese Plane Rectangular CS Number for specified epsg value");
    }

}


void createNodes(std::vector<autoware_map_msgs::Waypoint> awm_waypoints, std::vector<vector_map_msgs::Node> &vmap_nodes)
{
    for ( auto awm_wp : awm_waypoints)
    {
        vector_map_msgs::Node vmap_node;
        vmap_node.nid = awm_wp.waypoint_id;
        vmap_node.pid = awm_wp.waypoint_id;

        vmap_nodes.push_back(vmap_node);
    }
}

std::vector<int> findBranchingIdx(const std::vector<autoware_map_msgs::WaypointRelation> relation, int root_index)
{
    std::vector<int> branching_indices;

    for(auto itr = relation.begin(); itr != relation.end(); itr++)
    {
        if(itr->waypoint_id == root_index)
        {
            branching_indices.push_back(std::distance(relation.begin(), itr));
        }
    }
    return branching_indices;
}

std::vector<int> findMergingIdx(const std::vector<autoware_map_msgs::WaypointRelation> relation, int merged_index)
{
    std::vector<int> merging_indices;

    for(auto itr = relation.begin(); itr != relation.end(); itr++)
    {
        if(itr->next_waypoint_id == merged_index)
        {
            merging_indices.push_back(std::distance(relation.begin(), itr));
        }
    }
    return merging_indices;
}

int getJunctionType(const std::vector<autoware_map_msgs::WaypointRelation> awm_waypoint_relations,
                    std::vector<int> branching_idx,
                    std::vector<int> merging_idx)
{
    if(branching_idx.size() <= 1 && merging_idx.size() <= 1 )
    {
        return vector_map_msgs::Lane::NORMAL;
    }

    int left_branching_cnt = 0;
    int right_branching_cnt = 0;
    int left_merging_cnt = 0;
    int right_merging_cnt = 0;

    for(auto idx : branching_idx)
    {
        if( awm_waypoint_relations.at(idx).blinker == 1 )
        {
            left_branching_cnt++;
        }
        if( awm_waypoint_relations.at(idx).blinker == 2 )
        {
            right_branching_cnt++;
        }
    }

    for(auto idx : merging_idx)
    {
        if( awm_waypoint_relations.at(idx).blinker == 1 )
        {
            left_merging_cnt++;
        }
        if( awm_waypoint_relations.at(idx).blinker == 2 )
        {
            right_merging_cnt++;
        }
    }


    if( branching_idx.size() >= 3 || merging_idx.size() >= 3 || (branching_idx.size() >= 2 && merging_idx.size() >= 2) )
    {
        return vector_map_msgs::Lane::COMPOSITION;
    }

    if ( right_branching_cnt >= 1 )
    {
        return vector_map_msgs::Lane::RIGHT_BRANCHING;
    }
    if ( left_branching_cnt >= 1 )
    {
        return vector_map_msgs::Lane::LEFT_BRANCHING;
    }
    if ( right_merging_cnt >= 1 )
    {
        return vector_map_msgs::Lane::RIGHT_MERGING;
    }
    if ( left_merging_cnt >= 1 )
    {
        return vector_map_msgs::Lane::LEFT_MERGING;
    }

    ROS_ERROR_STREAM("could not find appropriate junction type!!!!!!!");
    return vector_map_msgs::Lane::NORMAL;
}

void createDTLanes(const std::vector<autoware_map_msgs::WaypointRelation> awm_waypoint_relations,
                   const autoware_map::AutowareMap awm,
                   std::vector<vector_map_msgs::DTLane> &vmap_dtlanes,
                   std::vector<vector_map_msgs::Lane> &vmap_lanes)
{
    unsigned int id = 1;
    for ( auto awm_waypoint_relation : awm_waypoint_relations)
    {
        vector_map_msgs::DTLane vmap_dtlane;
        vmap_dtlane.did = id++;
        vmap_dtlane.dist = awm_waypoint_relation.distance;
        vmap_dtlane.pid = awm_waypoint_relation.waypoint_id;

        autoware_map_msgs::Waypoint awm_waypoint = awm.findByKey( autoware_map::Key<autoware_map_msgs::Waypoint>(awm_waypoint_relation.waypoint_id));
        autoware_map_msgs::Waypoint awm_next_waypoint = awm.findByKey( autoware_map::Key<autoware_map_msgs::Waypoint>(awm_waypoint_relation.next_waypoint_id));

        vmap_dtlane.dir = awm_waypoint.yaw;
        vmap_dtlane.apara = 0;
        vmap_dtlane.r = 90000000000;

        autoware_map_msgs::Point pt1, pt2;
        pt1 = awm.findByKey(autoware_map::Key<autoware_map_msgs::Point>(awm_waypoint.point_id));
        pt2 = awm.findByKey(autoware_map::Key<autoware_map_msgs::Point>(awm_next_waypoint.point_id));
        double horizontal_dist = hypot(pt2.x - pt1.x, pt2.y - pt1.y);
        double vertical_dist = pt2.z - pt1.z;

        vmap_dtlane.slope = vertical_dist / horizontal_dist * 100;
        vmap_dtlane.cant = 0;
        vmap_dtlane.lw = awm_waypoint.width / 2;
        vmap_dtlane.rw = awm_waypoint.width / 2;

        vmap_dtlanes.push_back(vmap_dtlane);

        // std::vector<autoware_map_msgs::WaypointRelation> related_waypoint_relations= awm.findByFilter([&awm_waypoint_relation](const autoware_map_msgs::WaypointRelation& wpr){return wpr.waypoint_id == awm_waypoint_relation.waypoint_id;};

        vector_map_msgs::Lane vmap_lane;
        vmap_lane.lnid = id;
        vmap_lane.did = id;

        std::vector<int> merging_idx = findMergingIdx(awm_waypoint_relations, id);
        std::vector<int> branching_idx = findBranchingIdx(awm_waypoint_relations, id);

        //change order of branch/merge lanes according to blinkers. (staright < left turn < right turn)
        sort(merging_idx.begin(), merging_idx.end(), [&awm_waypoint_relations](const int x, const int y){ return awm_waypoint_relations.at(x).blinker < awm_waypoint_relations.at(y).blinker; });
        sort(branching_idx.begin(), branching_idx.end(), [&awm_waypoint_relations](const int x, const int y){ return awm_waypoint_relations.at(x).blinker < awm_waypoint_relations.at(y).blinker; });

        vmap_lane.jct = getJunctionType(awm_waypoint_relations, branching_idx, merging_idx);
        vmap_lane.blid = 0;
        vmap_lane.flid = 0;
        vmap_lane.blid2 = 0;
        vmap_lane.blid3 = 0;
        vmap_lane.blid4 = 0;
        vmap_lane.flid2 = 0;
        vmap_lane.flid3 = 0;
        vmap_lane.flid4 = 0;

        if(merging_idx.size() >= 1)
            vmap_lane.blid = merging_idx.at(0) + 1;
        if(merging_idx.size() >= 2)
            vmap_lane.blid2 = merging_idx.at(1) + 1;
        if(merging_idx.size() >= 3)
            vmap_lane.blid3 = merging_idx.at(2) + 1;
        if(merging_idx.size() >= 4)
            vmap_lane.blid4 = merging_idx.at(3) + 1;
        if(branching_idx.size() >= 1)
            vmap_lane.flid = merging_idx.at(0) + 1;
        if(branching_idx.size() >= 2)
            vmap_lane.flid2 = merging_idx.at(1) + 1;
        if(branching_idx.size() >= 3)
            vmap_lane.flid3 = merging_idx.at(2) + 1;
        if(branching_idx.size() >= 4)
            vmap_lane.flid4 = merging_idx.at(3) + 1;

        vmap_lane.bnid = awm_waypoint.point_id;
        vmap_lane.fnid = awm_next_waypoint.point_id;
        vmap_lane.span =  awm_waypoint_relation.distance;

        int awm_lane_id = 0;
        auto waypoint_lane_relation = awm.findByFilter([&](autoware_map_msgs::WaypointLaneRelation wlr){return wlr.waypoint_id == awm_waypoint.waypoint_id; });
        auto awm_lane = awm.findByKey( autoware_map::Key<autoware_map_msgs::Lane>(waypoint_lane_relation.front().lane_id) );
        vmap_lane.lcnt = awm_lane.num_of_lanes;
        vmap_lane.lanetype = awm_waypoint_relation.blinker;
        vmap_lane.limitvel = awm.findByKey(autoware_map::Key<autoware_map_msgs::Lane> (awm_lane_id)).speed_limit;
        vmap_lane.refvel = awm_waypoint.velocity;
        vmap_lane.lanecfgfg = (vmap_lane.lcnt > 1) ? 1 : 0;
        vmap_lane.lno = awm_lane.lane_number;
        vmap_lane.roadsecid = 0;
        vmap_lane.linkwaid = 0;
        vmap_lanes.push_back(vmap_lane);
    }
}

// void createRailCrossings(std::vector<autoware_map_msgs::LaneAttrRelation> awm_lane_attr_relations, std::vector<vector_map_msgs::RailCrossing> &vmap_rail_crossings){
//     unsigned int id = 1;
//     for ( auto awm_relation : awm_lane_attr_relations)
//     {
//         if( awm_relation.attribute_type == autoware_map::LaneAttrRelation::RAILROAD ) {
//             if(std::find_if(vmap_rail_crossings.begin(), vmap_rail_crossings.end(), [&](vector_map_msgs::RailCrossing rc){return rc.aid == awm_relation.area_id;}) != vmap_rail_crossings.end()){
//               continue;
//             }
//             vector_map_msgs::RailCrossing rail_crossing;
//             rail_crossing.id = id++;
//             rail_crossing.aid = awm_relation.area_id;
//             rail_crossing.linkid = 0;
//
//             vmap_rail_crossings.push_back(rail_crossing);
//         }
//     }
// }

void createWayAreas(std::vector<autoware_map_msgs::Wayarea> awm_wayareas, std::vector<vector_map_msgs::WayArea> &vmap_way_areas)
{
    for ( auto awm_area : awm_wayareas)
    {
        vector_map_msgs::WayArea way_area;
        way_area.waid = awm_area.wayarea_id;
        way_area.aid = awm_area.area_id;
        vmap_way_areas.push_back(way_area);

    }
}

void createSignals(     std::vector<autoware_map_msgs::SignalLight> awm_signal_lights,
                        std::vector<autoware_map_msgs::LaneSignalLightRelation> awm_lane_signal_relations,
                        std::vector<vector_map_msgs::Signal> &vmap_signals,
                        std::vector<vector_map_msgs::Vector> &vmap_vectors,
                        std::vector<vector_map_msgs::Pole> &vmap_dummy_poles,
                        const std::vector<autoware_map_msgs::WaypointRelation> awm_waypoint_relations,
                        const autoware_map::AutowareMap awm

                        )
{

    unsigned int vector_id = 1;
    for ( auto awm_signal_light : awm_signal_lights)
    {
        vector_map_msgs::Signal vmap_signal;

        vmap_signal.id = awm_signal_light.signal_light_id;
        //use signal_id as pole_id;
        vmap_signal.plid = awm_signal_light.signal_id;

        //create dummy poles
        vector_map_msgs::Pole vmap_pole;
        vmap_pole.plid = vmap_signal.plid;
        vmap_dummy_poles.push_back(vmap_pole);

        //color:{1 = red, 2=green, 3=yellow}
        if(awm_signal_light.color_type <=3 && awm_signal_light.arrow_type == 0)
        {
            vmap_signal.type = awm_signal_light.color_type;
        }
        else{
            vmap_signal.type = 9; //other
        }

        //create Vector to describe signal direction
        vector_map_msgs::Vector vmap_vector;
        vmap_vector.vid = vector_id;
        vmap_vector.pid = awm_signal_light.point_id;
        vmap_vector.hang =  awm_signal_light.horizontal_angle;
        vmap_vector.vang =  awm_signal_light.vertical_angle;
        vmap_vectors.push_back(vmap_vector);
        vmap_signal.vid = vector_id;
        vector_id += 1;


        auto lane_signal_itr = std::find_if(   awm_lane_signal_relations.begin(),
                                               awm_lane_signal_relations.end(),
                                               [awm_signal_light](autoware_map_msgs::LaneSignalLightRelation lslr){ return lslr.signal_light_id == awm_signal_light.signal_light_id; });

        auto awm_lane = awm.findByKey(autoware_map::Key<autoware_map_msgs::Lane>(lane_signal_itr->lane_id));

        int linkid = 0;
        for(auto itr = awm_waypoint_relations.begin(); itr != awm_waypoint_relations.end(); itr++)
        {
            if(itr->next_waypoint_id == awm_lane.end_waypoint_id)
            {
                auto awm_waypoint_lane_relations = awm.findByFilter([&](autoware_map_msgs::WaypointLaneRelation wlr){return wlr.waypoint_id == itr->waypoint_id; });
                for(auto awm_waypoint_lane_relation : awm_waypoint_lane_relations)
                {
                    if( awm_waypoint_lane_relation.lane_id == awm_lane.lane_id)
                    {
                        linkid = std::distance(awm_waypoint_relations.begin(), itr) + 1;
                    }
                }
            }
        }
        if(linkid == 0)
        {
            ROS_ERROR_STREAM("failed to find valid linkid for signal");
        }
        vmap_signal.linkid = linkid;
        vmap_signals.push_back(vmap_signal);
    }
}

double addAngles(double angle1, double angle2)
{
    double sum = angle1 + angle2;
    while( sum > M_PI ) sum -= 2 * M_PI;
    while( sum < M_PI ) sum += 2 * M_PI;
    return sum;
}

void createStopLines( const std::vector<autoware_map_msgs::Waypoint> awm_waypoints,
                      const std::vector<autoware_map_msgs::Point> awm_points,
                      const std::vector<autoware_map_msgs::WaypointRelation> awm_waypoint_relations,
                      std::vector<vector_map_msgs::Line> &vmap_lines,
                      std::vector<vector_map_msgs::Point> &vmap_points,
                      std::vector<vector_map_msgs::StopLine> &vmap_stop_lines

                      )
{
    int line_id = vmap_lines.size() + 1;
    int point_id = vmap_points.size() + 1;
    int stop_line_id = vmap_stop_lines.size() + 1;
    for(auto wp : awm_waypoints)
    {
        if(wp.stop_line == 1)
        {

            auto awm_pt = std::find_if(awm_points.begin(),
                                       awm_points.end(),
                                       [wp](autoware_map_msgs::Point pt){return pt.point_id == wp.point_id; });
            auto next_waypoint_relation = std::find_if(awm_waypoint_relations.begin(),
                                                       awm_waypoint_relations.end(),
                                                       [&](autoware_map_msgs::WaypointRelation wr){return wr.waypoint_id == wp.waypoint_id; });
            auto next_wp = std::find_if(awm_waypoints.begin(),
                                        awm_waypoints.end(),
                                        [&](autoware_map_msgs::Waypoint wp){return wp.waypoint_id == next_waypoint_relation->next_waypoint_id; });
            auto awm_next_pt = std::find_if(awm_points.begin(),
                                            awm_points.end(),
                                            [&](autoware_map_msgs::Point pt){return pt.point_id == next_wp->point_id; });
            double yaw = atan2(awm_next_pt->y - awm_pt->y, awm_next_pt->x - awm_pt->x);
            double angle_to_left = addAngles(yaw, -M_PI/2);
            double angle_to_right = addAngles(yaw, M_PI/2);
            double r = wp.width / 2;

            vector_map_msgs::Point start_point, end_point;
            start_point.pid = point_id++;
            //stop line must intersect with waypoints left side of the line
            start_point.bx = awm_pt->x + (r+0.1) * cos(angle_to_left);
            start_point.ly = awm_pt->y + (r+0.1) * sin(angle_to_left);
            start_point.h = awm_pt->z;
            //followings cannot be calculated from given information
            start_point.b = 0;
            start_point.l = 0;
            start_point.ref = 0;
            start_point.mcode1 = 0;
            start_point.mcode2 = 0;
            start_point.mcode3 = 0;
            vmap_points.push_back(start_point);

            end_point.pid = point_id++;
            //stop line must intersect with waypoints left side of the line
            end_point.bx = awm_pt->x + r * cos(angle_to_right);
            end_point.ly = awm_pt->y + r * sin(angle_to_right);
            end_point.h = awm_pt->z;
            //followings cannot be calculated from given information
            end_point.b = 0;
            end_point.l = 0;
            end_point.ref = 0;
            end_point.mcode1 = 0;
            end_point.mcode2 = 0;
            end_point.mcode3 = 0;
            vmap_points.push_back(end_point);

            vector_map_msgs::Line vmap_line;
            vmap_line.lid = line_id++;
            vmap_line.bpid = start_point.pid;
            vmap_line.fpid = end_point.pid;
            vmap_line.blid = vmap_line.flid = 0;
            vmap_lines.push_back(vmap_line);

            vector_map_msgs::StopLine vmap_stop_line;
            vmap_stop_line.id = stop_line_id++;
            vmap_stop_line.lid = vmap_line.lid;
            vmap_stop_line.tlid = 0;
            vmap_stop_line.signid = 1; //point to dummy road_sign

            auto relation = std::find_if(awm_waypoint_relations.begin(),
                                         awm_waypoint_relations.end(),
                                         [&](autoware_map_msgs::WaypointRelation wr){return wr.next_waypoint_id == wp.waypoint_id; });
            vmap_stop_line.linkid = std::distance(awm_waypoint_relations.begin(), relation) + 1;

            vmap_stop_lines.push_back(vmap_stop_line);
        }
    }
}
void createDummyRoadSign(std::vector<vector_map_msgs::RoadSign> &vmap_road_signs)
{

    vector_map_msgs::RoadSign road_sign;
    road_sign.id = 1;
    road_sign.vid = 0;
    road_sign.plid = 0;
    road_sign.type = 1;
    road_sign.linkid = 0;

    vmap_road_signs.push_back(road_sign);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "awm_vmap_converter");
    ros::NodeHandle nh;

    autoware_map::AutowareMap awm;
    awm.subscribe(nh, autoware_map::Category::ALL, ros::Duration(5));

    std::vector<autoware_map_msgs::Point> awm_points = awm.findByFilter([] (const autoware_map_msgs::Point){return true; });
    std::vector<vector_map_msgs::Point> vmap_points;
    createPoints(awm_points, vmap_points);
    std::vector<autoware_map_msgs::Waypoint> awm_waypoints = awm.findByFilter([] (const autoware_map_msgs::Waypoint){return true; });
    std::vector<vector_map_msgs::Node> vmap_nodes;
    createNodes(awm_waypoints, vmap_nodes);
    std::vector<autoware_map_msgs::Area> awm_areas = awm.findByFilter([] (const autoware_map_msgs::Area){return true; });
    std::vector<vector_map_msgs::Area> vmap_areas;
    std::vector<vector_map_msgs::Line> vmap_lines;
    createAreas(awm_areas, vmap_areas, vmap_lines);
    std::vector<autoware_map_msgs::WaypointRelation> awm_waypoint_relations = awm.findByFilter([] (const autoware_map_msgs::WaypointRelation){return true; });
    std::vector<vector_map_msgs::DTLane> vmap_dtlanes;
    std::vector<vector_map_msgs::Lane> vmap_lanes;
    createDTLanes(awm_waypoint_relations, awm, vmap_dtlanes,vmap_lanes);
    std::vector<autoware_map_msgs::LaneAttrRelation> awm_lane_attr_relations = awm.findByFilter([] (const autoware_map_msgs::LaneAttrRelation){return true; });
    std::vector<vector_map_msgs::CrossRoad> vmap_cross_roads;
    createCrossRoads(awm_lane_attr_relations, vmap_cross_roads);
    std::vector<vector_map_msgs::CrossWalk> vmap_cross_walks;
    createCrossWalks(awm_lane_attr_relations, vmap_cross_walks);
    // std::vector<vector_map_msgs::RailCrossing> vmap_rail_crossings;
    // createRailCrossings(awm_lane_attr_relations, vmap_rail_crossings);
    std::vector<autoware_map_msgs::Wayarea> awm_wayareas = awm.findByFilter([] (const autoware_map_msgs::Wayarea){return true; });
    std::vector<vector_map_msgs::WayArea> vmap_way_areas;
    createWayAreas(awm_wayareas, vmap_way_areas);
    std::vector<autoware_map_msgs::SignalLight> awm_signal_lights = awm.findByFilter([] (const autoware_map_msgs::SignalLight){return true; });
    std::vector<autoware_map_msgs::LaneSignalLightRelation> awm_lane_signal_relations = awm.findByFilter([] (const autoware_map_msgs::LaneSignalLightRelation){return true; });
    std::vector<vector_map_msgs::Signal> vmap_signals;
    std::vector<vector_map_msgs::Vector> vmap_vectors;
    std::vector<vector_map_msgs::Pole> vmap_dummy_poles;
    createSignals(  awm_signal_lights, awm_lane_signal_relations, vmap_signals, vmap_vectors,vmap_dummy_poles, awm_waypoint_relations, awm);
    std::vector<vector_map_msgs::StopLine> vmap_stop_lines;
    createStopLines( awm_waypoints, awm_points, awm_waypoint_relations, vmap_lines, vmap_points, vmap_stop_lines);
    std::vector<vector_map_msgs::RoadSign> vmap_road_signs;
    createDummyRoadSign(vmap_road_signs);

    ros::Publisher point_pub = nh.advertise<vector_map_msgs::PointArray>("vector_map_info/point", 1, true);
    ros::Publisher vector_pub = nh.advertise<vector_map_msgs::VectorArray>("vector_map_info/vector", 1, true);
    ros::Publisher line_pub = nh.advertise<vector_map_msgs::LineArray>("vector_map_info/line", 1, true);
    ros::Publisher area_pub = nh.advertise<vector_map_msgs::AreaArray>("vector_map_info/area", 1, true);
    ros::Publisher dtlane_pub = nh.advertise<vector_map_msgs::DTLaneArray>("vector_map_info/dtlane", 1, true);
    ros::Publisher node_pub = nh.advertise<vector_map_msgs::NodeArray>("vector_map_info/node", 1, true);
    ros::Publisher lane_pub = nh.advertise<vector_map_msgs::LaneArray>("vector_map_info/lane", 1, true);
    ros::Publisher way_area_pub = nh.advertise<vector_map_msgs::WayAreaArray>("vector_map_info/way_area", 1, true);
    ros::Publisher stop_line_pub = nh.advertise<vector_map_msgs::StopLineArray>("vector_map_info/stop_line", 1, true);
    ros::Publisher cross_walk_pub = nh.advertise<vector_map_msgs::CrossWalkArray>("vector_map_info/cross_walk", 1, true);
    ros::Publisher signal_pub = nh.advertise<vector_map_msgs::SignalArray>("vector_map_info/signal", 1, true);
    ros::Publisher pole_pub = nh.advertise<vector_map_msgs::PoleArray>("vector_map_info/pole", 1, true);
    // ros::Publisher side_walk_pub = nh.advertise<vector_map_msgs::SideWalkArray>("vector_map_info/side_walk", 1, true);
    ros::Publisher cross_road_pub = nh.advertise<vector_map_msgs::CrossRoadArray>("vector_map_info/cross_road", 1, true);
    // ros::Publisher rail_crossing_pub = nh.advertise<vector_map_msgs::RailCrossingArray>("vector_map_info/rail_crossing", 1, true);
    ros::Publisher road_sign_pub = nh.advertise<vector_map_msgs::RoadSignArray>("vector_map_info/road_sign", 1, true);

    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("vector_map", 1, true);
    ros::Publisher stat_pub = nh.advertise<std_msgs::Bool>("vmap_stat", 1, true);

    std_msgs::Bool stat;
    stat.data = false;
    stat_pub.publish(stat);


    vector_map::category_t category = vector_map::Category::NONE;
    if(!vmap_points.empty())
    {
        point_pub.publish( createObjectArray<vector_map_msgs::Point, vector_map_msgs::PointArray>(vmap_points));
        category |= vector_map::Category::POINT;
    }
    if(!vmap_vectors.empty())
    {
        vector_pub.publish( createObjectArray<vector_map_msgs::Vector, vector_map_msgs::VectorArray>(vmap_vectors));
        category |= vector_map::Category::VECTOR;
    }
    if(!vmap_lines.empty())
    {
        line_pub.publish( createObjectArray<vector_map_msgs::Line, vector_map_msgs::LineArray>(vmap_lines));
        category |= vector_map::Category::LINE;
    }
    if(!vmap_areas.empty())
    {
        area_pub.publish( createObjectArray<vector_map_msgs::Area, vector_map_msgs::AreaArray>(vmap_areas));
        category |= vector_map::Category::AREA;
    }
    if(!vmap_dtlanes.empty())
    {
        dtlane_pub.publish( createObjectArray<vector_map_msgs::DTLane, vector_map_msgs::DTLaneArray>(vmap_dtlanes));
        category |= vector_map::Category::DTLANE;
    }
    if(!vmap_nodes.empty())
    {
        node_pub.publish( createObjectArray<vector_map_msgs::Node, vector_map_msgs::NodeArray>(vmap_nodes));
        category |= vector_map::Category::NODE;
    }
    if(!vmap_lanes.empty())
    {
        lane_pub.publish( createObjectArray<vector_map_msgs::Lane, vector_map_msgs::LaneArray>(vmap_lanes));
        category |= vector_map::Category::LANE;
    }
    if(!vmap_way_areas.empty())
    {
        way_area_pub.publish( createObjectArray<vector_map_msgs::WayArea, vector_map_msgs::WayAreaArray>(vmap_way_areas));
        category |= vector_map::Category::WAY_AREA;
    }
    if(!vmap_stop_lines.empty())
    {
        stop_line_pub.publish( createObjectArray<vector_map_msgs::StopLine, vector_map_msgs::StopLineArray>(vmap_stop_lines));
        category |= vector_map::Category::STOP_LINE;
    }
    if(!vmap_cross_walks.empty())
    {
        cross_walk_pub.publish( createObjectArray<vector_map_msgs::CrossWalk, vector_map_msgs::CrossWalkArray>(vmap_cross_walks));
        category |= vector_map::Category::CROSS_WALK;
    }
    if(!vmap_signals.empty())
    {
        signal_pub.publish( createObjectArray<vector_map_msgs::Signal, vector_map_msgs::SignalArray>(vmap_signals));
        category |= vector_map::Category::SIGNAL;
    }
    if(!vmap_cross_roads.empty())
    {
        cross_road_pub.publish( createObjectArray<vector_map_msgs::CrossRoad, vector_map_msgs::CrossRoadArray>(vmap_cross_roads));
        category |= vector_map::Category::CROSS_ROAD;
    }
    // if(!vmap_rail_crossings.empty()){
    //   rail_crossing_pub.publish( createObjectArray<vector_map_msgs::RailCrossing, vector_map_msgs::RailCrossingArray>(vmap_rail_crossings));
    //   category |= vector_map::Category::RAIL_CROSSING;
    // }
    if(!vmap_dummy_poles.empty())
    {
        pole_pub.publish( createObjectArray<vector_map_msgs::Pole, vector_map_msgs::PoleArray>(vmap_dummy_poles));
        category |= vector_map::Category::POLE;
    }
    if(!vmap_road_signs.empty())
    {
        road_sign_pub.publish( createObjectArray<vector_map_msgs::RoadSign, vector_map_msgs::RoadSignArray>(vmap_road_signs));
        category |= vector_map::Category::ROAD_SIGN;
    }
    vector_map::VectorMap vmap;
    vmap.subscribe(nh, category);

    visualization_msgs::MarkerArray marker_array;
    insertMarkerArray(marker_array, createStopLineMarkerArray(vmap, vector_map::Color::WHITE));
    insertMarkerArray(marker_array, createCrossWalkMarkerArray(vmap, vector_map::Color::WHITE));
    insertMarkerArray(marker_array, createSignalMarkerArray(vmap, vector_map::Color::RED, vector_map::Color::BLUE, vector_map::Color::YELLOW, vector_map::Color::CYAN,
                                                            vector_map::Color::GRAY));
    insertMarkerArray(marker_array, createCrossRoadMarkerArray(vmap, vector_map::Color::LIGHT_GREEN));
    // insertMarkerArray(marker_array, createRailCrossingMarkerArray(vmap, vector_map::Color::LIGHT_MAGENTA));
    marker_array_pub.publish(marker_array);

    stat.data = true;
    stat_pub.publish(stat);
    ros::spin();
}

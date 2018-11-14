/*
 * VectorMapLoader.cpp
 *
 *  Created on: Nov 14, 2018
 *      Author: sujiwo
 */


#include <vector>
#include <string>
#include "VectorMapLoader.h"
#include "access_private.hpp"


using namespace std;
using namespace boost::filesystem;


using vector_map::VectorMap;
using vector_map::Category;
using vector_map::Color;
using vector_map::Key;

using vector_map::Point;
using vector_map::Vector;
using vector_map::Line;
using vector_map::Area;
using vector_map::Pole;
using vector_map::Box;
using vector_map::DTLane;
using vector_map::Node;
using vector_map::Lane;
using vector_map::WayArea;
using vector_map::RoadEdge;
using vector_map::Gutter;
using vector_map::Curb;
using vector_map::WhiteLine;
using vector_map::StopLine;
using vector_map::ZebraZone;
using vector_map::CrossWalk;
using vector_map::RoadMark;
using vector_map::RoadPole;
using vector_map::RoadSign;
using vector_map::Signal;
using vector_map::StreetLight;
using vector_map::UtilityPole;
using vector_map::GuardRail;
using vector_map::SideWalk;
using vector_map::DriveOnPortion;
using vector_map::CrossRoad;
using vector_map::SideStrip;
using vector_map::CurveMirror;
using vector_map::Wall;
using vector_map::Fence;
using vector_map::RailCrossing;

using vector_map::PointArray;
using vector_map::VectorArray;
using vector_map::LineArray;
using vector_map::AreaArray;
using vector_map::PoleArray;
using vector_map::BoxArray;
using vector_map::DTLaneArray;
using vector_map::NodeArray;
using vector_map::LaneArray;
using vector_map::WayAreaArray;
using vector_map::RoadEdgeArray;
using vector_map::GutterArray;
using vector_map::CurbArray;
using vector_map::WhiteLineArray;
using vector_map::StopLineArray;
using vector_map::ZebraZoneArray;
using vector_map::CrossWalkArray;
using vector_map::RoadMarkArray;
using vector_map::RoadPoleArray;
using vector_map::RoadSignArray;
using vector_map::SignalArray;
using vector_map::StreetLightArray;
using vector_map::UtilityPoleArray;
using vector_map::GuardRailArray;
using vector_map::SideWalkArray;
using vector_map::DriveOnPortionArray;
using vector_map::CrossRoadArray;
using vector_map::SideStripArray;
using vector_map::CurveMirrorArray;
using vector_map::WallArray;
using vector_map::FenceArray;
using vector_map::RailCrossingArray;

using vector_map::Handle;

/*
 * These function declarations are defined in vector_map.cpp, but somehow not exposed outside.
 */
namespace vector_map {
	void updatePoint(std::map<Key<Point>, Point>& map, const PointArray& msg);
}
bool isValidMarker(const visualization_msgs::Marker& marker);


const vector<string> VectorMapFileNames
    {
      "idx.csv",
      "point.csv",
      "vector.csv",
      "line.csv",
      "area.csv",
      "pole.csv",
      "box.csv",
      "dtlane.csv",
      "node.csv",
      "lane.csv",
      "wayarea.csv",
      "roadedge.csv",
      "gutter.csv",
      "curb.csv",
      "whiteline.csv",
      "stopline.csv",
      "zebrazone.csv",
      "crosswalk.csv",
      "road_surface_mark.csv",
      "poledata.csv",
      "roadsign.csv",
      "signaldata.csv",
      "streetlight.csv",
      "utilitypole.csv",
      "guardrail.csv",
      "sidewalk.csv",
      "driveon_portion.csv",
      "intersection.csv",
      "sidestrip.csv",
      "curvemirror.csv",
      "wall.csv",
      "fence.csv",
      "railroad_crossing.csv"
    };



void insertMarkerArray(visualization_msgs::MarkerArray& a1, const visualization_msgs::MarkerArray& a2)
{
	a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
}


visualization_msgs::Marker createLinkedLineMarker(const std::string& ns, int id, Color color, const VectorMap& vmap,
                                                  const Line& line)
{
  Area area;
  area.aid = 1; // must set valid aid
  area.slid = line.lid;
  return createAreaMarker(ns, id, color, vmap, area);
}

visualization_msgs::MarkerArray createRoadEdgeMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& road_edge : vmap.findByFilter([](const RoadEdge& road_edge){return true;}))
  {
    if (road_edge.lid == 0)
    {
      ROS_ERROR_STREAM("[createRoadEdgeMarkerArray] invalid road_edge: " << road_edge);
      continue;
    }

    Line line = vmap.findByKey(Key<Line>(road_edge.lid));
    if (line.lid == 0)
    {
      ROS_ERROR_STREAM("[createRoadEdgeMarkerArray] invalid line: " << line);
      continue;
    }

    if (line.blid == 0) // if beginning line
    {
      visualization_msgs::Marker marker = createLinkedLineMarker("road_edge", id++, color, vmap, line);
      if (isValidMarker(marker))
        marker_array.markers.push_back(marker);
      else
        ROS_ERROR_STREAM("[createRoadEdgeMarkerArray] failed createLinkedLineMarker: " << line);
    }
  }
  return marker_array;
}

visualization_msgs::MarkerArray createGutterMarkerArray(const VectorMap& vmap, Color no_cover_color,
                                                        Color cover_color, Color grating_color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& gutter : vmap.findByFilter([](const Gutter& gutter){return true;}))
  {
    if (gutter.aid == 0)
    {
      ROS_ERROR_STREAM("[createGutterMarkerArray] invalid gutter: " << gutter);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(gutter.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createGutterMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker;
    switch (gutter.type)
    {
    case Gutter::NO_COVER:
      marker = createAreaMarker("gutter", id++, no_cover_color, vmap, area);
      break;
    case Gutter::COVER:
      marker = createAreaMarker("gutter", id++, cover_color, vmap, area);
      break;
    case Gutter::GRATING:
      marker = createAreaMarker("gutter", id++, grating_color, vmap, area);
      break;
    default:
      ROS_ERROR_STREAM("[createGutterMarkerArray] unknown gutter.type: " << gutter);
      continue;
    }
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createGutterMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createCurbMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& curb : vmap.findByFilter([](const Curb& curb){return true;}))
  {
    if (curb.lid == 0)
    {
      ROS_ERROR_STREAM("[createCurbMarkerArray] invalid curb: " << curb);
      continue;
    }

    Line line = vmap.findByKey(Key<Line>(curb.lid));
    if (line.lid == 0)
    {
      ROS_ERROR_STREAM("[createCurbMarkerArray] invalid line: " << line);
      continue;
    }

    if (line.blid == 0) // if beginning line
    {
      visualization_msgs::Marker marker = createLinkedLineMarker("curb", id++, color, vmap, line);
      // XXX: The visualization_msgs::Marker::LINE_STRIP is difficult to deal with curb.width and curb.height.
      if (isValidMarker(marker))
        marker_array.markers.push_back(marker);
      else
        ROS_ERROR_STREAM("[createCurbMarkerArray] failed createLinkedLineMarker: " << line);
    }
  }
  return marker_array;
}

visualization_msgs::MarkerArray createWhiteLineMarkerArray(const VectorMap& vmap, Color white_color,
                                                           Color yellow_color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& white_line : vmap.findByFilter([](const WhiteLine& white_line){return true;}))
  {
    if (white_line.lid == 0)
    {
      ROS_ERROR_STREAM("[createWhiteLineMarkerArray] invalid white_line: " << white_line);
      continue;
    }
    if (white_line.type == WhiteLine::DASHED_LINE_BLANK) // if invisible line
      continue;

    Line line = vmap.findByKey(Key<Line>(white_line.lid));
    if (line.lid == 0)
    {
      ROS_ERROR_STREAM("[createWhiteLineMarkerArray] invalid line: " << line);
      continue;
    }

    if (line.blid == 0) // if beginning line
    {
      visualization_msgs::Marker marker;
      switch (white_line.color)
      {
      case 'W':
        marker = createLinkedLineMarker("white_line", id++, white_color, vmap, line);
        break;
      case 'Y':
        marker = createLinkedLineMarker("white_line", id++, yellow_color, vmap, line);
        break;
      default:
        ROS_ERROR_STREAM("[createWhiteLineMarkerArray] unknown white_line.color: " << white_line);
        continue;
      }
      // XXX: The visualization_msgs::Marker::LINE_STRIP is difficult to deal with white_line.width.
      if (isValidMarker(marker))
        marker_array.markers.push_back(marker);
      else
        ROS_ERROR_STREAM("[createWhiteLineMarkerArray] failed createLinkedLineMarker: " << line);
    }
  }
  return marker_array;
}

visualization_msgs::MarkerArray createStopLineMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& stop_line : vmap.findByFilter([](const StopLine& stop_line){return true;}))
  {
    if (stop_line.lid == 0)
    {
      ROS_ERROR_STREAM("[createStopLineMarkerArray] invalid stop_line: " << stop_line);
      continue;
    }

    Line line = vmap.findByKey(Key<Line>(stop_line.lid));
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

visualization_msgs::MarkerArray createZebraZoneMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& zebra_zone : vmap.findByFilter([](const ZebraZone& zebra_zone){return true;}))
  {
    if (zebra_zone.aid == 0)
    {
      ROS_ERROR_STREAM("[createZebraZoneMarkerArray] invalid zebra_zone: " << zebra_zone);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(zebra_zone.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createZebraZoneMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("zebra_zone", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createZebraZoneMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createCrossWalkMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& cross_walk : vmap.findByFilter([](const CrossWalk& cross_walk){return true;}))
  {
    if (cross_walk.aid == 0)
    {
      ROS_ERROR_STREAM("[createCrossWalkMarkerArray] invalid cross_walk: " << cross_walk);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(cross_walk.aid));
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

visualization_msgs::MarkerArray createRoadMarkMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& road_mark : vmap.findByFilter([](const RoadMark& road_mark){return true;}))
  {
    if (road_mark.aid == 0)
    {
      ROS_ERROR_STREAM("[createRoadMarkMarkerArray] invalid road_mark: " << road_mark);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(road_mark.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createRoadMarkMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("road_mark", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createRoadMarkMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createRoadPoleMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& road_pole : vmap.findByFilter([](const RoadPole& road_pole){return true;}))
  {
    if (road_pole.plid == 0)
    {
      ROS_ERROR_STREAM("[createRoadPoleMarkerArray] invalid road_pole: " << road_pole);
      continue;
    }

    Pole pole = vmap.findByKey(Key<Pole>(road_pole.plid));
    if (pole.plid == 0)
    {
      ROS_ERROR_STREAM("[createRoadPoleMarkerArray] invalid pole: " << pole);
      continue;
    }

    visualization_msgs::Marker marker = createPoleMarker("road_pole", id++, color, vmap, pole);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createRoadPoleMarkerArray] failed createPoleMarker: " << pole);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createRoadSignMarkerArray(const VectorMap& vmap, Color sign_color, Color pole_color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& road_sign : vmap.findByFilter([](const RoadSign& road_sign){return true;}))
  {
    if (road_sign.vid == 0)
    {
      ROS_ERROR_STREAM("[createRoadSignMarkerArray] invalid road_sign: " << road_sign);
      continue;
    }

    Vector vector = vmap.findByKey(Key<Vector>(road_sign.vid));
    if (vector.vid == 0)
    {
      ROS_ERROR_STREAM("[createRoadSignMarkerArray] invalid vector: " << vector);
      continue;
    }

    Pole pole;
    if (road_sign.plid != 0)
    {
      pole = vmap.findByKey(Key<Pole>(road_sign.plid));
      if (pole.plid == 0)
      {
        ROS_ERROR_STREAM("[createRoadSignMarkerArray] invalid pole: " << pole);
        continue;
      }
    }

    visualization_msgs::Marker vector_marker = createVectorMarker("road_sign", id++, sign_color, vmap, vector);
    if (isValidMarker(vector_marker))
      marker_array.markers.push_back(vector_marker);
    else
      ROS_ERROR_STREAM("[createRoadSignMarkerArray] failed createVectorMarker: " << vector);

    if (road_sign.plid != 0)
    {
      visualization_msgs::Marker pole_marker = createPoleMarker("road_sign", id++, pole_color, vmap, pole);
      if (isValidMarker(pole_marker))
        marker_array.markers.push_back(pole_marker);
      else
        ROS_ERROR_STREAM("[createRoadSignMarkerArray] failed createPoleMarker: " << pole);
    }
  }
  return marker_array;
}

visualization_msgs::MarkerArray createSignalMarkerArray(const VectorMap& vmap, Color red_color, Color blue_color,
                                                        Color yellow_color, Color other_color, Color pole_color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& signal : vmap.findByFilter([](const Signal& signal){return true;}))
  {
    if (signal.vid == 0)
    {
      ROS_ERROR_STREAM("[createSignalMarkerArray] invalid signal: " << signal);
      continue;
    }

    Vector vector = vmap.findByKey(Key<Vector>(signal.vid));
    if (vector.vid == 0)
    {
      ROS_ERROR_STREAM("[createSignalMarkerArray] invalid vector: " << vector);
      continue;
    }

    Pole pole;
    if (signal.plid != 0)
    {
      pole = vmap.findByKey(Key<Pole>(signal.plid));
      if (pole.plid == 0)
      {
        ROS_ERROR_STREAM("[createSignalMarkerArray] invalid pole: " << pole);
        continue;
      }
    }

    visualization_msgs::Marker vector_marker;
    switch (signal.type)
    {
    case Signal::RED:
    case Signal::PEDESTRIAN_RED:
      vector_marker = createVectorMarker("signal", id++, red_color, vmap, vector);
      break;
    case Signal::BLUE:
    case Signal::PEDESTRIAN_BLUE:
      vector_marker = createVectorMarker("signal", id++, blue_color, vmap, vector);
      break;
    case Signal::YELLOW:
      vector_marker = createVectorMarker("signal", id++, yellow_color, vmap, vector);
      break;
    case Signal::RED_LEFT:
      vector_marker = createVectorMarker("signal", id++, Color::LIGHT_RED, vmap, vector);
          break;
    case Signal::BLUE_LEFT:
      vector_marker = createVectorMarker("signal", id++, Color::LIGHT_GREEN, vmap, vector);
          break;
    case Signal::YELLOW_LEFT:
      vector_marker = createVectorMarker("signal", id++, Color::LIGHT_YELLOW, vmap, vector);
          break;
    case Signal::OTHER:
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

    if (signal.plid != 0)
    {
      visualization_msgs::Marker pole_marker = createPoleMarker("signal", id++, pole_color, vmap, pole);
      if (isValidMarker(pole_marker))
        marker_array.markers.push_back(pole_marker);
      else
        ROS_ERROR_STREAM("[createSignalMarkerArray] failed createPoleMarker: " << pole);
    }
  }
  return marker_array;
}

visualization_msgs::MarkerArray createStreetLightMarkerArray(const VectorMap& vmap, Color light_color,
                                                             Color pole_color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& street_light : vmap.findByFilter([](const StreetLight& street_light){return true;}))
  {
    if (street_light.lid == 0)
    {
      ROS_ERROR_STREAM("[createStreetLightMarkerArray] invalid street_light: " << street_light);
      continue;
    }

    Line line = vmap.findByKey(Key<Line>(street_light.lid));
    if (line.lid == 0)
    {
      ROS_ERROR_STREAM("[createStreetLightMarkerArray] invalid line: " << line);
      continue;
    }

    Pole pole;
    if (street_light.plid != 0)
    {
      pole = vmap.findByKey(Key<Pole>(street_light.plid));
      if (pole.plid == 0)
      {
        ROS_ERROR_STREAM("[createStreetLightMarkerArray] invalid pole: " << pole);
        continue;
      }
    }

    if (line.blid == 0) // if beginning line
    {
      visualization_msgs::Marker line_marker = createLinkedLineMarker("street_light", id++, light_color, vmap, line);
      if (isValidMarker(line_marker))
        marker_array.markers.push_back(line_marker);
      else
        ROS_ERROR_STREAM("[createStreetLightMarkerArray] failed createLinkedLineMarker: " << line);
    }

    if (street_light.plid != 0)
    {
      visualization_msgs::Marker pole_marker = createPoleMarker("street_light", id++, pole_color, vmap, pole);
      if (isValidMarker(pole_marker))
        marker_array.markers.push_back(pole_marker);
      else
        ROS_ERROR_STREAM("[createStreetLightMarkerArray] failed createPoleMarker: " << pole);
    }
  }
  return marker_array;
}

visualization_msgs::MarkerArray createUtilityPoleMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& utility_pole : vmap.findByFilter([](const UtilityPole& utility_pole){return true;}))
  {
    if (utility_pole.plid == 0)
    {
      ROS_ERROR_STREAM("[createUtilityPoleMarkerArray] invalid utility_pole: " << utility_pole);
      continue;
    }

    Pole pole = vmap.findByKey(Key<Pole>(utility_pole.plid));
    if (pole.plid == 0)
    {
      ROS_ERROR_STREAM("[createUtilityPoleMarkerArray] invalid pole: " << pole);
      continue;
    }

    visualization_msgs::Marker marker = createPoleMarker("utility_pole", id++, color, vmap, pole);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createUtilityPoleMarkerArray] failed createPoleMarker: " << pole);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createGuardRailMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& guard_rail : vmap.findByFilter([](const GuardRail& guard_rail){return true;}))
  {
    if (guard_rail.aid == 0)
    {
      ROS_ERROR_STREAM("[createGuardRailMarkerArray] invalid guard_rail: " << guard_rail);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(guard_rail.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createGuardRailMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("guard_rail", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createGuardRailMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createSideWalkMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& side_walk : vmap.findByFilter([](const SideWalk& side_walk){return true;}))
  {
    if (side_walk.aid == 0)
    {
      ROS_ERROR_STREAM("[createSideWalkMarkerArray] invalid side_walk: " << side_walk);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(side_walk.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createSideWalkMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("side_walk", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createSideWalkMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createDriveOnPortionMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& drive_on_portion : vmap.findByFilter([](const DriveOnPortion& drive_on_portion){return true;}))
  {
    if (drive_on_portion.aid == 0)
    {
      ROS_ERROR_STREAM("[createDriveOnPortionMarkerArray] invalid drive_on_portion: " << drive_on_portion);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(drive_on_portion.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createDriveOnPortionMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("drive_on_portion", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createDriveOnPortionMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createCrossRoadMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& cross_road : vmap.findByFilter([](const CrossRoad& cross_road){return true;}))
  {
    if (cross_road.aid == 0)
    {
      ROS_ERROR_STREAM("[createCrossRoadMarkerArray] invalid cross_road: " << cross_road);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(cross_road.aid));
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

visualization_msgs::MarkerArray createSideStripMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& side_strip : vmap.findByFilter([](const SideStrip& side_strip){return true;}))
  {
    if (side_strip.lid == 0)
    {
      ROS_ERROR_STREAM("[createSideStripMarkerArray] invalid side_strip: " << side_strip);
      continue;
    }

    Line line = vmap.findByKey(Key<Line>(side_strip.lid));
    if (line.lid == 0)
    {
      ROS_ERROR_STREAM("[createSideStripMarkerArray] invalid line: " << line);
      continue;
    }

    if (line.blid == 0) // if beginning line
    {
      visualization_msgs::Marker marker = createLinkedLineMarker("side_strip", id++, color, vmap, line);
      if (isValidMarker(marker))
        marker_array.markers.push_back(marker);
      else
        ROS_ERROR_STREAM("[createSideStripMarkerArray] failed createLinkedLineMarker: " << line);
    }
  }
  return marker_array;
}

visualization_msgs::MarkerArray createCurveMirrorMarkerArray(const VectorMap& vmap, Color mirror_color,
                                                             Color pole_color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& curve_mirror : vmap.findByFilter([](const CurveMirror& curve_mirror){return true;}))
  {
    if (curve_mirror.vid == 0 || curve_mirror.plid == 0)
    {
      ROS_ERROR_STREAM("[createCurveMirrorMarkerArray] invalid curve_mirror: " << curve_mirror);
      continue;
    }

    Vector vector = vmap.findByKey(Key<Vector>(curve_mirror.vid));
    if (vector.vid == 0)
    {
      ROS_ERROR_STREAM("[createCurveMirrorMarkerArray] invalid vector: " << vector);
      continue;
    }

    Pole pole = vmap.findByKey(Key<Pole>(curve_mirror.plid));
    if (pole.plid == 0)
    {
      ROS_ERROR_STREAM("[createCurveMirrorMarkerArray] invalid pole: " << pole);
      continue;
    }

    visualization_msgs::Marker vector_marker = createVectorMarker("curve_mirror", id++, mirror_color, vmap, vector);
    if (isValidMarker(vector_marker))
      marker_array.markers.push_back(vector_marker);
    else
      ROS_ERROR_STREAM("[createCurveMirrorMarkerArray] failed createVectorMarker: " << vector);

    visualization_msgs::Marker pole_marker = createPoleMarker("curve_mirror", id++, pole_color, vmap, pole);
    if (isValidMarker(pole_marker))
      marker_array.markers.push_back(pole_marker);
    else
      ROS_ERROR_STREAM("[createCurveMirrorMarkerArray] failed createPoleMarker: " << pole);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createWallMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& wall : vmap.findByFilter([](const Wall& wall){return true;}))
  {
    if (wall.aid == 0)
    {
      ROS_ERROR_STREAM("[createWallMarkerArray] invalid wall: " << wall);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(wall.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createWallMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("wall", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createWallMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createFenceMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& fence : vmap.findByFilter([](const Fence& fence){return true;}))
  {
    if (fence.aid == 0)
    {
      ROS_ERROR_STREAM("[createFenceMarkerArray] invalid fence: " << fence);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(fence.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createFenceMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("fence", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createFenceMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}

visualization_msgs::MarkerArray createRailCrossingMarkerArray(const VectorMap& vmap, Color color)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  for (const auto& rail_crossing : vmap.findByFilter([](const RailCrossing& rail_crossing){return true;}))
  {
    if (rail_crossing.aid == 0)
    {
      ROS_ERROR_STREAM("[createRailCrossingMarkerArray] invalid rail_crossing: " << rail_crossing);
      continue;
    }

    Area area = vmap.findByKey(Key<Area>(rail_crossing.aid));
    if (area.aid == 0)
    {
      ROS_ERROR_STREAM("[createRailCrossingMarkerArray] invalid area: " << area);
      continue;
    }

    visualization_msgs::Marker marker = createAreaMarker("rail_crossing", id++, color, vmap, area);
    if (isValidMarker(marker))
      marker_array.markers.push_back(marker);
    else
      ROS_ERROR_STREAM("[createRailCrossingMarkerArray] failed createAreaMarker: " << area);
  }
  return marker_array;
}


template <class T, class U>
U createObjectArray(const std::string& file_path)
{
	U obj_array;
	// NOTE: Autoware want to use map messages with or without /use_sim_time.
	// Therefore we don't set obj_array.header.stamp.
	// obj_array.header.stamp = ros::Time::now();
	obj_array.header.frame_id = "map";
	obj_array.data = vector_map::parse<T>(file_path);
	return obj_array;
}


VectorMapLoader::VectorMapLoader(const std::string &directory) :
	vmDir(directory)
{
	if (!is_directory(vmDir))
		throw runtime_error("Not a directory");

	loadAll ();
}

// Strange bits for accessing private members
typedef	vector_map::Handle<	Point	,	PointArray	>	HandlePoint	;
typedef	vector_map::Handle<	Vector	,	VectorArray	>	HandleVector	;
typedef	vector_map::Handle<	Line	,	LineArray	>	HandleLine	;
typedef	vector_map::Handle<	Area	,	AreaArray	>	HandleArea	;
typedef	vector_map::Handle<	Pole	,	PoleArray	>	HandlePole	;
typedef	vector_map::Handle<	Box	,	BoxArray	>	HandleBox	;
typedef	vector_map::Handle<	DTLane	,	DTLaneArray	>	HandleDTLane	;
typedef	vector_map::Handle<	Node	,	NodeArray	>	HandleNode	;
typedef	vector_map::Handle<	Lane	,	LaneArray	>	HandleLane	;
typedef	vector_map::Handle<	WayArea	,	WayAreaArray	>	HandleWayArea	;
typedef	vector_map::Handle<	RoadEdge	,	RoadEdgeArray	>	HandleRoadEdge	;
typedef	vector_map::Handle<	Gutter	,	GutterArray	>	HandleGutter	;
typedef	vector_map::Handle<	Curb	,	CurbArray	>	HandleCurb	;
typedef	vector_map::Handle<	WhiteLine	,	WhiteLineArray	>	HandleWhiteLine	;
typedef	vector_map::Handle<	StopLine	,	StopLineArray	>	HandleStopLine	;
typedef	vector_map::Handle<	ZebraZone	,	ZebraZoneArray	>	HandleZebraZone	;
typedef	vector_map::Handle<	CrossWalk	,	CrossWalkArray	>	HandleCrossWalk	;
typedef	vector_map::Handle<	RoadMark	,	RoadMarkArray	>	HandleRoadMark	;
typedef	vector_map::Handle<	RoadPole	,	RoadPoleArray	>	HandleRoadPole	;
typedef	vector_map::Handle<	RoadSign	,	RoadSignArray	>	HandleRoadSign	;
typedef	vector_map::Handle<	Signal	,	SignalArray	>	HandleSignal	;
typedef	vector_map::Handle<	StreetLight	,	StreetLightArray	>	HandleStreetLight	;
typedef	vector_map::Handle<	UtilityPole	,	UtilityPoleArray	>	HandleUtilityPole	;
typedef	vector_map::Handle<	GuardRail	,	GuardRailArray	>	HandleGuardRail	;
typedef	vector_map::Handle<	SideWalk	,	SideWalkArray	>	HandleSideWalk	;
typedef	vector_map::Handle<	DriveOnPortion	,	DriveOnPortionArray	>	HandleDriveOnPortion	;
typedef	vector_map::Handle<	CrossRoad	,	CrossRoadArray	>	HandleCrossRoad	;
typedef	vector_map::Handle<	SideStrip	,	SideStripArray	>	HandleSideStrip	;
typedef	vector_map::Handle<	CurveMirror	,	CurveMirrorArray	>	HandleCurveMirror	;
typedef	vector_map::Handle<	Wall	,	WallArray	>	HandleWall	;
typedef	vector_map::Handle<	Fence	,	FenceArray	>	HandleFence	;
typedef	vector_map::Handle<	RailCrossing	,	RailCrossingArray	>	HandleRailCrossing	;

ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandlePoint, point_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleVector, vector_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleLine, line_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleArea, area_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandlePole, pole_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleBox, box_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleDTLane, dtlane_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleNode, node_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleLane, lane_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleWayArea, way_area_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleRoadEdge, road_edge_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleGutter, gutter_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleCurb, curb_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleWhiteLine, white_line_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleStopLine, stop_line_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleZebraZone, zebra_zone_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleCrossWalk, cross_walk_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleRoadMark, road_mark_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleRoadPole, road_pole_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleRoadSign, road_sign_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleSignal, signal_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleStreetLight, street_light_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleUtilityPole, utility_pole_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleGuardRail, guard_rail_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleSideWalk, side_walk_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleDriveOnPortion, drive_on_portion_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleCrossRoad, cross_road_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleSideStrip, side_strip_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleCurveMirror, curve_mirror_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleWall, wall_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleFence, fence_);
ACCESS_PRIVATE_FIELD(vector_map::VectorMap, HandleRailCrossing, rail_crossing_);


ACCESS_PRIVATE_FUN(HandlePoint, void(const PointArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleVector, void(const VectorArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleLine, void(const LineArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleArea, void(const AreaArray&), subscribe);
ACCESS_PRIVATE_FUN(HandlePole, void(const PoleArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleBox, void(const BoxArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleDTLane, void(const DTLaneArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleNode, void(const NodeArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleLane, void(const LaneArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleWayArea, void(const WayAreaArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleRoadEdge, void(const RoadEdgeArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleGutter, void(const GutterArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleCurb, void(const CurbArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleWhiteLine, void(const WhiteLineArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleStopLine, void(const StopLineArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleZebraZone, void(const ZebraZoneArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleCrossWalk, void(const CrossWalkArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleRoadMark, void(const RoadMarkArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleRoadPole, void(const RoadPoleArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleRoadSign, void(const RoadSignArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleSignal, void(const SignalArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleStreetLight, void(const StreetLightArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleUtilityPole, void(const UtilityPoleArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleGuardRail, void(const GuardRailArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleSideWalk, void(const SideWalkArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleDriveOnPortion, void(const DriveOnPortionArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleCrossRoad, void(const CrossRoadArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleSideStrip, void(const SideStripArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleCurveMirror, void(const CurveMirrorArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleWall, void(const WallArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleFence, void(const FenceArray&), subscribe);
ACCESS_PRIVATE_FUN(HandleRailCrossing, void(const RailCrossingArray&), subscribe);


void VectorMapLoader::loadAll()
{
	for (directory_entry &fe: directory_iterator(vmDir)) {
		string file_name = basename(fe);
		if (file_name == "idx.csv")
		{
			; // XXX: This version of Autoware don't support index csv file now.
		}
		else if (file_name == "point.csv")
		{
			auto objArray = createObjectArray<Point, PointArray>(fe.path().string());
			call_private::subscribe(access_private::point_(*this), objArray);
		}

	    else if (file_name == "vector.csv")
	    {
	    	auto objArray = createObjectArray<Vector, VectorArray>(fe.path().string());
	    	call_private::subscribe(access_private::vector_(*this), objArray);
	    }
	    else if (file_name == "line.csv")
	    {
	    	auto objArray = createObjectArray<Line, LineArray>(fe.path().string());
	    	call_private::subscribe(access_private::line_(*this), objArray);
	    }
	    else if (file_name == "area.csv")
	    {
	    	auto objArray = createObjectArray<Area, AreaArray>(fe.path().string());
	    	call_private::subscribe(access_private::area_(*this), objArray);
	    }
	    else if (file_name == "pole.csv")
	    {
	    	auto objArray = createObjectArray<Pole, PoleArray>(fe.path().string());
	    	call_private::subscribe(access_private::pole_(*this), objArray);
	    }
	    else if (file_name == "box.csv")
	    {
	    	auto objArray = createObjectArray<Box, BoxArray>(fe.path().string());
	    	call_private::subscribe(access_private::box_(*this), objArray);
	    }
	    else if (file_name == "dtlane.csv")
	    {
	    	auto objArray = createObjectArray<DTLane, DTLaneArray>(fe.path().string());
	    	call_private::subscribe(access_private::dtlane_(*this), objArray);
	    }
	    else if (file_name == "node.csv")
	    {
	    	auto objArray = createObjectArray<Node, NodeArray>(fe.path().string());
	    	call_private::subscribe(access_private::node_(*this), objArray);
	    }
	    else if (file_name == "lane.csv")
	    {
	    	auto objArray = createObjectArray<Lane, LaneArray>(fe.path().string());
	    	call_private::subscribe(access_private::lane_(*this), objArray);
	    }
	    else if (file_name == "wayarea.csv")
	    {
	    	auto objArray = createObjectArray<WayArea, WayAreaArray>(fe.path().string());
	    	call_private::subscribe(access_private::way_area_(*this), objArray);
	    }
	    else if (file_name == "roadedge.csv")
	    {
	    	auto objArray = createObjectArray<RoadEdge, RoadEdgeArray>(fe.path().string());
	    	call_private::subscribe(access_private::road_edge_(*this), objArray);
	    }
	    else if (file_name == "gutter.csv")
	    {
	    	auto objArray = createObjectArray<Gutter, GutterArray>(fe.path().string());
	    	call_private::subscribe(access_private::gutter_(*this), objArray);
	    }
	    else if (file_name == "curb.csv")
	    {
	    	auto objArray = createObjectArray<Curb, CurbArray>(fe.path().string());
	    	call_private::subscribe(access_private::curb_(*this), objArray);
	    }
	    else if (file_name == "whiteline.csv")
	    {
	    	auto objArray = createObjectArray<WhiteLine, WhiteLineArray>(fe.path().string());
	    	call_private::subscribe(access_private::white_line_(*this), objArray);
	    }
	    else if (file_name == "stopline.csv")
	    {
	    	auto objArray = createObjectArray<StopLine, StopLineArray>(fe.path().string());
	    	call_private::subscribe(access_private::stop_line_(*this), objArray);
	    }
	    else if (file_name == "zebrazone.csv")
	    {
	    	auto objArray = createObjectArray<ZebraZone, ZebraZoneArray>(fe.path().string());
	    	call_private::subscribe(access_private::zebra_zone_(*this), objArray);
	    }
	    else if (file_name == "crosswalk.csv")
	    {
	    	auto objArray = createObjectArray<CrossWalk, CrossWalkArray>(fe.path().string());
	    	call_private::subscribe(access_private::cross_walk_(*this), objArray);
	    }
	    else if (file_name == "road_surface_mark.csv")
	    {
	    	auto objArray = createObjectArray<RoadMark, RoadMarkArray>(fe.path().string());
	    	call_private::subscribe(access_private::road_mark_(*this), objArray);
	    }
	    else if (file_name == "poledata.csv")
	    {
	    	auto objArray = createObjectArray<RoadPole, RoadPoleArray>(fe.path().string());
	    	call_private::subscribe(access_private::road_pole_(*this), objArray);
	    }
	    else if (file_name == "roadsign.csv")
	    {
	    	auto objArray = createObjectArray<RoadSign, RoadSignArray>(fe.path().string());
	    	call_private::subscribe(access_private::road_sign_(*this), objArray);
	    }
	    else if (file_name == "signaldata.csv")
	    {
	    	auto objArray = createObjectArray<Signal, SignalArray>(fe.path().string());
	    	call_private::subscribe(access_private::signal_(*this), objArray);
	    }
	    else if (file_name == "streetlight.csv")
	    {
	    	auto objArray = createObjectArray<StreetLight, StreetLightArray>(fe.path().string());
	    	call_private::subscribe(access_private::street_light_(*this), objArray);
	    }
	    else if (file_name == "utilitypole.csv")
	    {
	    	auto objArray = createObjectArray<UtilityPole, UtilityPoleArray>(fe.path().string());
	    	call_private::subscribe(access_private::utility_pole_(*this), objArray);
	    }
	    else if (file_name == "guardrail.csv")
	    {
	    	auto objArray = createObjectArray<GuardRail, GuardRailArray>(fe.path().string());
	    	call_private::subscribe(access_private::guard_rail_(*this), objArray);
	    }
	    else if (file_name == "sidewalk.csv")
	    {
	    	auto objArray = createObjectArray<SideWalk, SideWalkArray>(fe.path().string());
	    	call_private::subscribe(access_private::side_walk_(*this), objArray);
	    }
	    else if (file_name == "driveon_portion.csv")
	    {
	    	auto objArray = createObjectArray<DriveOnPortion, DriveOnPortionArray>(fe.path().string());
	    	call_private::subscribe(access_private::drive_on_portion_(*this), objArray);
	    }
	    else if (file_name == "intersection.csv")
	    {
	    	auto objArray = createObjectArray<CrossRoad, CrossRoadArray>(fe.path().string());
	    	call_private::subscribe(access_private::cross_road_(*this), objArray);
	    }
	    else if (file_name == "sidestrip.csv")
	    {
	    	auto objArray = createObjectArray<SideStrip, SideStripArray>(fe.path().string());
	    	call_private::subscribe(access_private::side_strip_(*this), objArray);
	    }
	    else if (file_name == "curvemirror.csv")
	    {
	    	auto objArray = createObjectArray<CurveMirror, CurveMirrorArray>(fe.path().string());
	    	call_private::subscribe(access_private::curve_mirror_(*this), objArray);
	    }
	    else if (file_name == "wall.csv")
	    {
	    	auto objArray = createObjectArray<Wall, WallArray>(fe.path().string());
	    	call_private::subscribe(access_private::wall_(*this), objArray);
	    }
	    else if (file_name == "fence.csv")
	    {
	    	auto objArray = createObjectArray<Fence, FenceArray>(fe.path().string());
	    	call_private::subscribe(access_private::fence_(*this), objArray);
	    }
	    else if (file_name == "railroad_crossing.csv")
	    {
	    	auto objArray = createObjectArray<RailCrossing, RailCrossingArray>(fe.path().string());
	    	call_private::subscribe(access_private::rail_crossing_(*this), objArray);
	    }

	}

	// Do insertion to marker_array here
	insertMarkerArray(marker_array, createRoadEdgeMarkerArray(*this, Color::GRAY));
	insertMarkerArray(marker_array, createGutterMarkerArray(*this, Color::GRAY, Color::GRAY, Color::GRAY));
	insertMarkerArray(marker_array, createCurbMarkerArray(*this, Color::GRAY));
	insertMarkerArray(marker_array, createWhiteLineMarkerArray(*this, Color::WHITE, Color::YELLOW));
	insertMarkerArray(marker_array, createStopLineMarkerArray(*this, Color::WHITE));
	insertMarkerArray(marker_array, createZebraZoneMarkerArray(*this, Color::WHITE));
	insertMarkerArray(marker_array, createCrossWalkMarkerArray(*this, Color::WHITE));
	insertMarkerArray(marker_array, createRoadMarkMarkerArray(*this, Color::WHITE));
	insertMarkerArray(marker_array, createRoadPoleMarkerArray(*this, Color::GRAY));
	insertMarkerArray(marker_array, createRoadSignMarkerArray(*this, Color::GREEN, Color::GRAY));
	insertMarkerArray(marker_array, createSignalMarkerArray(*this, Color::RED, Color::BLUE, Color::YELLOW, Color::CYAN,
														  Color::GRAY));
	insertMarkerArray(marker_array, createStreetLightMarkerArray(*this, Color::YELLOW, Color::GRAY));
	insertMarkerArray(marker_array, createUtilityPoleMarkerArray(*this, Color::GRAY));
	insertMarkerArray(marker_array, createGuardRailMarkerArray(*this, Color::LIGHT_BLUE));
	insertMarkerArray(marker_array, createSideWalkMarkerArray(*this, Color::GRAY));
	insertMarkerArray(marker_array, createDriveOnPortionMarkerArray(*this, Color::LIGHT_CYAN));
	insertMarkerArray(marker_array, createCrossRoadMarkerArray(*this, Color::LIGHT_GREEN));
	insertMarkerArray(marker_array, createSideStripMarkerArray(*this, Color::GRAY));
	insertMarkerArray(marker_array, createCurveMirrorMarkerArray(*this, Color::MAGENTA, Color::GRAY));
	insertMarkerArray(marker_array, createWallMarkerArray(*this, Color::LIGHT_YELLOW));
	insertMarkerArray(marker_array, createFenceMarkerArray(*this, Color::LIGHT_RED));
	insertMarkerArray(marker_array, createRailCrossingMarkerArray(*this, Color::LIGHT_MAGENTA));

}



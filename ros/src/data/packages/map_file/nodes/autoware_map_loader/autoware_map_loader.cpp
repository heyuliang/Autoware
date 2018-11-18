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

#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_map/autoware_map.h>
#include <sys/stat.h>

#include <autoware_map_msgs/PointArray.h>


namespace
{
void printUsage()
{
    ROS_ERROR_STREAM("Usage:");
    ROS_ERROR_STREAM("rosrun map_file vector_map_loader [CSV]...");
}

template <class T, class U>
U createObjectArray(const std::string& file_path)
{
    U obj_array;
    // NOTE: Autoware want to use map messages with or without /use_sim_time.
    // Therefore we don't set obj_array.header.stamp.
    // obj_array.header.stamp = ros::Time::now();
    obj_array.header.frame_id = "map";
    obj_array.data = autoware_map::parse<T>(file_path);
    return obj_array;
}

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "autoware_map_loader");
    ros::NodeHandle nh;

    if (argc < 2)
    {
        printUsage();
        return EXIT_FAILURE;
    }

    ros::Publisher lane_pub = nh.advertise<autoware_map_msgs::LaneArray>("autoware_map_info/lane", 1, true);
    ros::Publisher lane_attr_relation_pub = nh.advertise<autoware_map_msgs::LaneAttrRelationArray>("autoware_map_info/lane_attr_relation", 1, true);
    ros::Publisher lane_relation_pub = nh.advertise<autoware_map_msgs::LaneRelationArray>("autoware_map_info/lane_relation", 1, true);
    ros::Publisher lane_signal_light_relation_pub = nh.advertise<autoware_map_msgs::LaneSignalLightRelationArray>("autoware_map_info/lane_signal_light_relation", 1, true);
    ros::Publisher lane_change_relation_pub = nh.advertise<autoware_map_msgs::LaneChangeRelationArray>("autoware_map_info/lane_change_relation", 1, true);
    ros::Publisher opposite_lane_relation_pub = nh.advertise<autoware_map_msgs::OppositeLaneRelationArray>("autoware_map_info/opposite_lane_relation", 1, true);
    ros::Publisher point_pub = nh.advertise<autoware_map_msgs::PointArray>("autoware_map_info/point", 1, true);
    ros::Publisher area_pub = nh.advertise<autoware_map_msgs::AreaArray>("autoware_map_info/area", 1, true);
    ros::Publisher route_pub = nh.advertise<autoware_map_msgs::RouteArray>("autoware_map_info/route", 1, true);
    ros::Publisher signal_pub = nh.advertise<autoware_map_msgs::SignalArray>("autoware_map_info/signal", 1, true);
    ros::Publisher signal_light_pub = nh.advertise<autoware_map_msgs::SignalLightArray>("autoware_map_info/signal_light", 1, true);
    ros::Publisher wayarea_pub = nh.advertise<autoware_map_msgs::WayareaArray>("autoware_map_info/wayarea", 1, true);
    ros::Publisher waypoint_pub = nh.advertise<autoware_map_msgs::WaypointArray>("autoware_map_info/waypoint", 1, true);
    ros::Publisher waypoint_lane_relation_pub = nh.advertise<autoware_map_msgs::WaypointLaneRelationArray>("autoware_map_info/waypoint_lane_relation", 1, true);
    ros::Publisher waypoint_relation_pub = nh.advertise<autoware_map_msgs::WaypointRelationArray>("autoware_map_info/waypoint_relation", 1, true);
    ros::Publisher waypoint_signal_relation_pub = nh.advertise<autoware_map_msgs::WaypointSignalRelationArray>("autoware_map_info/waypoint_signal_relation", 1, true);
    // ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("vector_map", 1, true);
    ros::Publisher stat_pub = nh.advertise<std_msgs::Bool>("awm_stat", 1, true);

    std_msgs::Bool stat;
    stat.data = false;
    stat_pub.publish(stat);

    std::vector<std::string> file_paths;
    for (int i = 1; i < argc; ++i)
    {
        std::string file_path(argv[i]);
        file_paths.push_back(file_path);
    }

    // vector_map::category_t category = Category::NONE;
    for (const auto& file_path : file_paths)
    {
        std::string file_name(basename(file_path.c_str()));
        if (file_name == "point.csv")
        {
            point_pub.publish(createObjectArray<autoware_map_msgs::Point, autoware_map_msgs::PointArray>(file_path));
            // category |= Category::POINT;
        }
        else if(file_name == "lane.csv")
        {
            lane_pub.publish(createObjectArray<autoware_map_msgs::Lane, autoware_map_msgs::LaneArray>(file_path));
        }
        else if(file_name == "lane_attribute_relations.csv")
        {
            lane_attr_relation_pub.publish(createObjectArray<autoware_map_msgs::LaneAttrRelation, autoware_map_msgs::LaneAttrRelationArray>(file_path));
        }
        else if(file_name == "lane_relations.csv")
        {
            lane_relation_pub.publish(createObjectArray<autoware_map_msgs::LaneRelation, autoware_map_msgs::LaneRelationArray>(file_path));
        }
        else if(file_name == "lane_signal_light_relations.csv")
        {
            lane_signal_light_relation_pub.publish(createObjectArray<autoware_map_msgs::LaneSignalLightRelation, autoware_map_msgs::LaneSignalLightRelationArray>(file_path));
        }
        else if(file_name == "lane_change_relations.csv" )
        {
            lane_change_relation_pub.publish(createObjectArray<autoware_map_msgs::LaneChangeRelation, autoware_map_msgs::LaneChangeRelationArray>(file_path));
        }
        else if(file_name == "opposite_lane_relations.csv")
        {
            opposite_lane_relation_pub.publish(createObjectArray<autoware_map_msgs::OppositeLaneRelation, autoware_map_msgs::OppositeLaneRelationArray>(file_path));
        }
        else if(file_name == "area.csv")
        {
            area_pub.publish(createObjectArray<autoware_map_msgs::Area, autoware_map_msgs::AreaArray>(file_path));
        }
        else if(file_name == "route.csv")
        {
            route_pub.publish(createObjectArray<autoware_map_msgs::Route, autoware_map_msgs::RouteArray>(file_path));
        }
        else if(file_name == "signal.csv")
        {
            signal_pub.publish(createObjectArray<autoware_map_msgs::Signal, autoware_map_msgs::SignalArray>(file_path));
        }
        else if(file_name == "signal_light.csv")
        {
            signal_light_pub.publish(createObjectArray<autoware_map_msgs::SignalLight, autoware_map_msgs::SignalLightArray>(file_path));
        }
        else if(file_name == "wayarea.csv")
        {
            wayarea_pub.publish(createObjectArray<autoware_map_msgs::Wayarea, autoware_map_msgs::WayareaArray>(file_path));
        }
        else if(file_name == "waypoint.csv")
        {
            waypoint_pub.publish(createObjectArray<autoware_map_msgs::Waypoint, autoware_map_msgs::WaypointArray>(file_path));
        }
        else if(file_name == "waypoint_lane_relations.csv")
        {
            waypoint_lane_relation_pub.publish(createObjectArray<autoware_map_msgs::WaypointLaneRelation, autoware_map_msgs::WaypointLaneRelationArray>(file_path));
        }
        else if(file_name == "waypoint_relations.csv")
        {
            waypoint_relation_pub.publish(createObjectArray<autoware_map_msgs::WaypointRelation, autoware_map_msgs::WaypointRelationArray>(file_path));
        }
        else if(file_name == "waypoint_signal_relations.csv")
        {
            waypoint_signal_relation_pub.publish(createObjectArray<autoware_map_msgs::WaypointSignalRelation, autoware_map_msgs::WaypointSignalRelationArray>(file_path));
        }
        else
            ROS_ERROR_STREAM("unknown csv file: " << file_path);
    }

    stat.data = true;
    stat_pub.publish(stat);

    ros::spin();

    return EXIT_SUCCESS;
}

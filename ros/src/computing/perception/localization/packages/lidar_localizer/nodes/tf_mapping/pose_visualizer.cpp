#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <random>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

std::vector<std::string> split(std::string &input, char delimiter) {
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

ros::Time stoRosTime(std::string &input) {
  ros::Time time;
  std::vector<std::string> str_vec = split(input, '.');
  time.sec = std::stoi(str_vec.at(0));
  time.nsec = std::stoi(str_vec.at(1));

  return time;
}

// Remove extension from filename
std::string removeExtension(const std::string &filename) {
  std::string::size_type pos;
  if ((pos = filename.find_last_of(".")) == std::string::npos) {
    return filename;
  }
  return filename.substr(0, pos);
}

int main(int argc, char **argv) {

  std::cout << "argc: " << argc << std::endl;
  for(int i = 0; i < argc; i++){
   std::cout << "argv[" << i << "]: " << argv[i] << std::endl;
  }

  if (argc < 2) {
    std::cout << "rosrun lidar_localizer pose_visualizer 'pose_file'" << std::endl;
    return -1;
  }

  ros::init(argc, argv, "pose_visualizer");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::srand(time(NULL));

  std::vector<ros::Publisher> marker_array_pub_array;
  for(int i = 0; i < argc-1; i++){
    ros::Publisher marker_array_pub;
    marker_array_pub_array.push_back(marker_array_pub);
    std::string marker_topic = "/" + removeExtension(argv[i+1]) + "/pose_marker";
    marker_array_pub_array[i] = nh.advertise<visualization_msgs::MarkerArray>(marker_topic, 1, true);
  }

  for(int i = 0; i < argc-1; i++) {

    std::string pose_filename = argv[i+1];
    std::ifstream ifs(pose_filename);
    if (!ifs) {
      std::cout << "Couldn't open " << pose_filename << "." << std::endl;
      exit(1);
    }

    int count = 0;

    visualization_msgs::MarkerArray marker_array;
    std::string line;

    std_msgs::ColorRGBA color;
    color.r = ((double) std::rand() / RAND_MAX);
    color.g = ((double) std::rand() / RAND_MAX);
    color.b = ((double) std::rand() / RAND_MAX);
    color.a = 0.8;

    std::cout << color.r << std::endl;

    while (getline(ifs, line)) {

      geometry_msgs::Point point;
      geometry_msgs::Quaternion orientation;
      tf::Quaternion q;

      std::vector <std::string> str_vec = split(line, ',');
      ros::Time stamp = stoRosTime(str_vec.at(0));
      point.x = std::stod(str_vec.at(1));
      point.y = std::stod(str_vec.at(2));
      point.z = std::stod(str_vec.at(3));
      q.setRPY(std::stod(str_vec.at(4)), std::stod(str_vec.at(5)), std::stod(str_vec.at(6)));
      tf::quaternionTFToMsg(q, orientation);

      visualization_msgs::Marker marker;
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time();
      marker.ns = "marker";
      marker.id = count;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration();

      marker.color = color;

      marker.pose.position = point;
      marker.pose.orientation = orientation;

      marker.scale.x = 1.0; // arrow length
      marker.scale.y = 0.5;  // arrow width
      marker.scale.z = 0.5;

      marker_array.markers.push_back(marker);

      count++;
    }

    marker_array_pub_array[i].publish(marker_array);
  }

  ros::spin();

  return 0;
}

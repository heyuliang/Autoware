#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <random>
#include <list>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/*
std::vector<std::string> split(std::string &input, char delimiter) {
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}
 */
/*
ros::Time stoRosTime(std::string &input) {
  ros::Time time;
  std::vector<std::string> str_vec = split(input, '.');
  time.sec = std::stoi(str_vec.at(0));
  time.nsec = std::stoi(str_vec.at(1));

  return time;
}
*/
// Remove extension from filename
std::string removeExtension(const std::string &filename) {
  std::string::size_type pos;
  if ((pos = filename.find_last_of(".")) == std::string::npos) {
    return filename;
  }
  return filename.substr(0, pos);
}

std::istream& safeGetline(std::istream& is, std::string& t)
{
  t.clear();

  // The characters in the stream are read one-by-one using a std::streambuf.
  // That is faster than reading them one-by-one using the std::istream.
  // Code that uses streambuf this way must be guarded by a sentry object.
  // The sentry object performs various tasks,
  // such as thread synchronization and updating the stream state.

  std::istream::sentry se(is, true);
  std::streambuf* sb = is.rdbuf();

  for(;;) {
    int c = sb->sbumpc();
    switch (c) {
      case '\n':
        return is;
      case '\r':
        if(sb->sgetc() == '\n')
          sb->sbumpc();
        return is;
      case std::streambuf::traits_type::eof():
        // Also handle the case when the last line has no line ending
        if(t.empty())
          is.setstate(std::ios::eofbit);
        return is;
      default:
        t += (char)c;
    }
  }
}

void addNewAxes(visualization_msgs::MarkerArray& marker_array, geometry_msgs::Point position, tf::Quaternion orientation)
{
  std_msgs::ColorRGBA color_x, color_y, color_z;
  color_x.r = 1.0;
  color_x.g = 0.0;
  color_x.b = 0.0;
  color_x.a = 0.5;

  color_y.r = 0.0;
  color_y.g = 1.0;
  color_y.b = 0.0;
  color_y.a = 0.5;

  color_z.r = 0.0;
  color_z.g = 0.0;
  color_z.b = 1.0;
  color_z.a = 0.5;

  tf::Quaternion x_to_y, x_to_z, y_to_z;
  x_to_y.setRPY(0.0, 0.0, -M_PI / 2.0);
  x_to_z.setRPY(0.0, M_PI / 2.0, 0.0);
  y_to_z.setRPY(M_PI / 2.0, 0.0, 0.0);

  visualization_msgs::Marker marker_x, marker_y, marker_z;

  marker_x.header.frame_id = "/map";
  marker_x.header.stamp = ros::Time();
  marker_x.ns = "x_axis";
  marker_x.id = marker_array.markers.size();
  marker_x.type = visualization_msgs::Marker::ARROW;
  marker_x.action = visualization_msgs::Marker::ADD;
  marker_x.lifetime = ros::Duration();

  marker_x.color = color_x;

  marker_x.pose.position = position;
  tf::quaternionTFToMsg(orientation, marker_x.pose.orientation);

  marker_x.scale.x = 0.5; // arrow length
  marker_x.scale.y = 0.1;  // arrow width
  marker_x.scale.z = 0.1;

  marker_array.markers.push_back(marker_x);

  marker_y.header.frame_id = "/map";
  marker_y.header.stamp = ros::Time();
  marker_y.ns = "y_axis";
  marker_y.id = marker_array.markers.size();
  marker_y.type = visualization_msgs::Marker::ARROW;
  marker_y.action = visualization_msgs::Marker::ADD;
  marker_y.lifetime = ros::Duration();

  marker_y.color = color_y;

  marker_y.pose.position = position;
  tf::quaternionTFToMsg(orientation * x_to_y, marker_y.pose.orientation);

  marker_y.scale.x = 0.5; // arrow length
  marker_y.scale.y = 0.1;  // arrow width
  marker_y.scale.z = 0.1;

  marker_array.markers.push_back(marker_y);

  marker_z.header.frame_id = "/map";
  marker_z.header.stamp = ros::Time();
  marker_z.ns = "z_axis";
  marker_z.id = marker_array.markers.size();
  marker_z.type = visualization_msgs::Marker::ARROW;
  marker_z.action = visualization_msgs::Marker::ADD;
  marker_z.lifetime = ros::Duration();

  marker_z.color = color_z;

  marker_z.pose.position = position;
  tf::quaternionTFToMsg(orientation * x_to_z, marker_z.pose.orientation);

  marker_z.scale.x = 0.5; // arrow length
  marker_z.scale.y = 0.1;  // arrow width
  marker_z.scale.z = 0.1;

  marker_array.markers.push_back(marker_z);

}

int main(int argc, char **argv) {

  std::cout << "argc: " << argc << std::endl;
  for(int i = 0; i < argc; i++){
   std::cout << "argv[" << i << "]: " << argv[i] << std::endl;
  }

  if (argc < 2) {
    std::cout << "rosrun lidar_localizer pospac_visualizer 'pospac_file'" << std::endl;
    return -1;
  }

  ros::init(argc, argv, "pospac_visualizer");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::srand(time(NULL));

  std::vector<ros::Publisher> marker_array_pub_array;
  for(int i = 0; i < argc-1; i++){
    ros::Publisher marker_array_pub;
    marker_array_pub_array.push_back(marker_array_pub);
//    std::string marker_topic = "/" + removeExtension(argv[i+1]) + "/pose_marker";
    std::string marker_topic = "/pose_marker";
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

    while (getline(ifs, line)) {
      if(count <= 33){
        count++;
        continue;
      }
      count++;

      if(count % 10 == 0) {
        geometry_msgs::Point point;
        geometry_msgs::Quaternion orientation;
        tf::Quaternion q;

        std::string delim(" ");
        std::list <std::string> list_str;
        boost::split(list_str, line, boost::is_any_of(delim), boost::algorithm::token_compress_on);
/*
      BOOST_FOREACH(std::string s, list_str){
        std::cout << s << std::endl;
      }
*/
        // Convert list to vector
        std::vector <std::string> vec_str;
        for (std::list<std::string>::iterator iter = list_str.begin(); iter != list_str.end(); iter++) {
          vec_str.push_back(*iter);
        }
        /*
        for (int i = 0; i < (int)vec_str.size(); i++){
          std::cout << "i: " << i << " " << vec_str.at(i) << std::endl;
        }
         */
//      ros::Time stamp = stoRosTime(list_vec.at(0));
        ros::Time stamp = ros::Time::now();

        point.x = std::stod(vec_str.at(3));
        point.y = std::stod(vec_str.at(4));
        point.z = std::stod(vec_str.at(5));
        double roll = std::stod(vec_str.at(9)) * M_PI / 180.0;
        double pitch = -1.0 * std::stod(vec_str.at(10)) * M_PI / 180.0;
        double yaw = -1.0 * std::stod(vec_str.at(11)) * M_PI / 180.0 + M_PI / 2.0;
        q.setRPY(roll, pitch, yaw);

        addNewAxes(marker_array, point, q);
      }
//      count++;
    }

    marker_array_pub_array[i].publish(marker_array);
    std::cout << "Published." << std::endl;
  }

  ros::spin();

  return 0;
}

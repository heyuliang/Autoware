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
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "pospac_datatypes.h"

static std::vector<pospac_export> vec_pospac_export;

std::vector<std::string> split(std::string &input, char delimiter) {
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

ros::Time stringToRosTime(std::string &input) {
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




pospac_export findPospacDataByStamp(std::vector<pospac_export>& vec_pospac, ros::Time stamp)
{
  pospac_export ret;
  ros::Duration diff;
  ros::Duration d(0.01);
  for(std::vector<pospac_export>::iterator iter = vec_pospac.begin(); iter != vec_pospac.end(); iter++)
  {
    diff = stamp - iter->timestamp;
    if(diff < d)
    {
      ret = *iter;
      break;
    }
  }
  return ret;
}

void points_callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &input)
{
  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::PointCloud<pcl::PointXYZI> gps_transformed_scan, submap;

  std_msgs::Header header;
  pcl_conversions::fromPCL(input->header, header);

  // remove points within 1.0m
  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator iter = input->begin(); iter != input->end(); iter++)
  {
    p.x = (double)iter->x;
    p.y = (double)iter->y;
    p.z = (double)iter->z;
    p.intensity = (double)iter->intensity;

    double r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0) + pow(p.z, 2.0));
    if (r > 1.0)
    {
      scan.push_back(p);
    }
  }

  pospac_export data = findPospacDataByStamp(vec_pospac_export, header.stamp);
  std::cout << "header.stamp: " << header.stamp << std::endl;
  std::cout << "data.timestamp: " << data.timestamp << std::endl;

  static tf::TransformBroadcaster br;
  tf::Vector3 v(data.position.x, data.position.y, data.position.z);
  tf::Quaternion q;
  tf::quaternionMsgToTF(data.orientation, q);
  tf::Transform transform(q, v);

  br.sendTransform(tf::StampedTransform(transform, header.stamp, "/map", "/gps"));
  std::cout << "Transform sent." << std::endl;

  // transform scan using gps transform
  /*
  if(scan.points.size() > 0) {
    try {
      tf_listener_gps->waitForTransform("map", "base_link", header.stamp, ros::Duration(1));
      tf_listener_gps->lookupTransform("map", "base_link", header.stamp, gps_transform);

      static tf::StampedTransform test;

      tf_listener_test->waitForTransform("map", "velodyne", header.stamp, ros::Duration(1));
      tf_listener_test->lookupTransform("map", "velodyne", header.stamp, test);

      pcl_ros::transformPointCloud(scan, gps_transformed_scan, test);
      tf::Transform tmp = gps_transform * tf_base_link_to_velodyne;


      double ro, pi, ya;
      std::cout << "gps_transform.getOrigin().getX(): " << gps_transform.getOrigin().getX() << std::endl;
      std::cout << "gps_transform.getOrigin().getY(): " << gps_transform.getOrigin().getY() << std::endl;
      std::cout << "gps_transform.getOrigin().getZ(): " << gps_transform.getOrigin().getZ() << std::endl;
      tf::Matrix3x3 tf(gps_transform.getRotation());
      tf.getRPY(ro,pi,ya);
      std::cout << ro << std::endl;
      std::cout << pi << std::endl;
      std::cout << ya << std::endl;

      std::cout << "tmp.getOrigin().getX(): " << tmp.getOrigin().getX() << std::endl;
      std::cout << "tmp.getOrigin().getY(): " << tmp.getOrigin().getY() << std::endl;
      std::cout << "tmp.getOrigin().getZ(): " << tmp.getOrigin().getZ() << std::endl;
      tf::Matrix3x3 tf2(tmp.getRotation());

      tf2.getRPY(ro,pi,ya);
      std::cout << ro << std::endl;
      std::cout << pi << std::endl;
      std::cout << ya << std::endl;

      std::cout << "test.getOrigin().getX(): " << test.getOrigin().getX() << std::endl;
      std::cout << "test.getOrigin().getY(): " << test.getOrigin().getY() << std::endl;
      std::cout << "test.getOrigin().getZ(): " << test.getOrigin().getZ() << std::endl;
      tf::Matrix3x3 tf3(test.getRotation());

      tf3.getRPY(ro,pi,ya);
      std::cout << ro << std::endl;
      std::cout << pi << std::endl;
      std::cout << ya << std::endl;


    }
    catch (tf::TransformException ex) {
      std::cout << "Transform not found" << std::endl;
      return;
    }
  }

  map_gps += gps_transformed_scan;

  if (!ofs_map_gps)
  {
    std::cerr << "Could not open " << filename_map_gps << "." << std::endl;
    exit(1);
  }

  for (int i = 0; i < (int)gps_transformed_scan.points.size(); i++)
  {
    ofs_map_gps << std::fixed << std::setprecision(10) << gps_transformed_scan.points[i].x << ","
                << std::fixed << std::setprecision(10) << gps_transformed_scan.points[i].y << ","
                << std::fixed << std::setprecision(10) << gps_transformed_scan.points[i].z << ","
                << gps_transformed_scan.points[i].intensity << std::endl;
  }

  std::cout << "Wrote " << gps_transformed_scan.size() << " points to " << filename_map_gps << "." << std::endl;

  if(isFirstScan == true){
    map_slam += gps_transformed_scan;
    isFirstScan = false;
  }

  // ndt scan matching
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

  double gps_x, gps_y, gps_z, gps_roll, gps_pitch, gps_yaw;
  gps_x = gps_transform.getOrigin().x();
  gps_y = gps_transform.getOrigin().y();
  gps_z = gps_transform.getOrigin().z();
  tf::Matrix3x3(gps_transform.getRotation()).getRPY(gps_roll, gps_pitch, gps_yaw);


  if (!ofs_pose_gps)
  {
    std::cerr << "Could not open " << filename_pose_slam << "." << std::endl;
    exit(1);
  }

  double gps_pose_x, gps_pose_y, gps_pose_z, gps_pose_roll, gps_pose_pitch, gps_pose_yaw;
  gps_pose_x = gps_transform.getOrigin().x();
  gps_pose_y = gps_transform.getOrigin().y();
  gps_pose_z = gps_transform.getOrigin().z();
  tf::Matrix3x3(gps_transform.getRotation()).getRPY(gps_pose_roll, gps_pose_pitch, gps_pose_yaw);

  ofs_pose_gps << gps_transform.stamp_ << ","
               << std::fixed << std::setprecision(10) << gps_pose_x << ","
               << std::fixed << std::setprecision(10) << gps_pose_y << ","
               << std::fixed << std::setprecision(10) << gps_pose_z << ","
               << std::fixed << std::setprecision(10) << gps_pose_roll << ","
               << std::fixed << std::setprecision(10) << gps_pose_pitch << ","
               << std::fixed << std::setprecision(10) << gps_pose_yaw << std::endl;

  if(USE_SLAM == true && map_slam.points.size() != 0) {
    ndt.setTransformationEpsilon(trans_eps);
    ndt.setStepSize(step_size);
    ndt.setResolution(ndt_res);
    ndt.setMaximumIterations(max_iter);
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_slam_ptr(new pcl::PointCloud<pcl::PointXYZI>(map_slam));
    ndt.setInputTarget(map_slam_ptr);

    double voxel_leaf_size = 1.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    // apply voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);

    ndt.setInputSource(filtered_scan_ptr);

    // Initial pose for scan matching
    Eigen::AngleAxisf init_rotation_x(gps_roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(gps_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(gps_yaw, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(gps_x, gps_y, gps_z);

    Eigen::Matrix4f init_guess =
        (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * mat_base_link_to_velodyne;

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    ndt.align(*output_cloud, init_guess);

    bool has_converged = ndt.hasConverged();
    int final_num_iteration = ndt.getFinalNumIteration();
    double fitness_score = ndt.getFitnessScore();
    double transformation_probability = ndt.getTransformationProbability();
    Eigen::Matrix4f t_velodyne = ndt.getFinalTransformation();
    Eigen::Matrix4f t_base_link = t_velodyne * mat_base_link_to_velodyne.inverse();

    tf::Matrix3x3 mat_velodyne, mat_base_link;

    mat_velodyne.setValue(static_cast<double>(t_velodyne(0, 0)), static_cast<double>(t_velodyne(0, 1)),
                          static_cast<double>(t_velodyne(0, 2)), static_cast<double>(t_velodyne(1, 0)),
                          static_cast<double>(t_velodyne(1, 1)), static_cast<double>(t_velodyne(1, 2)),
                          static_cast<double>(t_velodyne(2, 0)), static_cast<double>(t_velodyne(2, 1)),
                          static_cast<double>(t_velodyne(2, 2)));

    mat_base_link.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                           static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                           static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                           static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                           static_cast<double>(t_base_link(2, 2)));

    ndt_pose_msg.header.frame_id = "map";
    ndt_pose_msg.header.stamp = header.stamp;
    ndt_pose_msg.pose.position.x = t_base_link(0,3);
    ndt_pose_msg.pose.position.y = t_base_link(1,3);
    ndt_pose_msg.pose.position.z = t_base_link(2,3);
    tf::Quaternion q;
    mat_base_link.getRotation(q);
    tf::quaternionTFToMsg(q, ndt_pose_msg.pose.orientation);
    ndt_pose_pub.publish(ndt_pose_msg);

    double roll, pitch, yaw;
    mat_base_link.getRPY(roll, pitch, yaw);

    velodyne_pose_msg.header.frame_id = "map";
    velodyne_pose_msg.header.stamp = header.stamp;
    velodyne_pose_msg.pose.position.x = t_velodyne(0,3);
    velodyne_pose_msg.pose.position.y = t_velodyne(1,3);
    velodyne_pose_msg.pose.position.z = t_velodyne(2,3);
    tf::Quaternion q2;
    mat_velodyne.getRotation(q2);
    tf::quaternionTFToMsg(q2, velodyne_pose_msg.pose.orientation);
    velodyne_pose_pub.publish(velodyne_pose_msg);

    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Number of map points: " << map_slam_ptr->points.size() << std::endl;
    std::cout << "Number of scan points: " << input->points.size() << std::endl;
    std::cout << "Number of filtered scan points: " << filtered_scan_ptr->points.size() << std::endl;
    std::cout << "NDT has converged: " << has_converged << std::endl;
    std::cout << "Number of iteration: " << final_num_iteration << std::endl;
    std::cout << "Fitness score: " << fitness_score << std::endl;
    std::cout << "Transformation Probability: " << transformation_probability << std::endl;
    std::cout << "Final Transformation:" << std::endl;
    std::cout << t_base_link << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr slam_transformed_scan(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(scan, *slam_transformed_scan, t_velodyne);

    std::cout << "yaw - previous_yaw: " << yaw - previous_yaw << std::endl;
    if(abs(yaw - previous_yaw) <= 0.1) {
      map_slam += *slam_transformed_scan;


      if (!ofs_map_slam) {
        std::cerr << "Could not open " << filename_map_slam << "." << std::endl;
        exit(1);
      }

      for (int i = 0; i < (int) slam_transformed_scan->points.size(); i++) {
        ofs_map_slam << std::fixed << std::setprecision(10) << slam_transformed_scan->points[i].x << ","
                     << std::fixed << std::setprecision(10) << slam_transformed_scan->points[i].y << ","
                     << std::fixed << std::setprecision(10) << slam_transformed_scan->points[i].z << ","
                     << slam_transformed_scan->points[i].intensity << std::endl;
      }

      std::cout << "Wrote " << slam_transformed_scan->points.size() << " points to " << filename_map_slam << "."
                << std::endl;
    }


    if (!ofs_pose_slam)
    {
      std::cerr << "Could not open " << filename_pose_slam << "." << std::endl;
      exit(1);
    }

    ofs_pose_slam << gps_transform.stamp_ << ","
                  << std::fixed << std::setprecision(10) << ndt_pose_msg.pose.position.x << ","
                  << std::fixed << std::setprecision(10) << ndt_pose_msg.pose.position.y << ","
                  << std::fixed << std::setprecision(10) << ndt_pose_msg.pose.position.z << ","
                  << std::fixed << std::setprecision(10) << roll << ","
                  << std::fixed << std::setprecision(10) << pitch << ","
                  << std::fixed << std::setprecision(10) << yaw << std::endl;

    // publish map
    while ((int) queue_submap.size() > WINDOW_SIZE) {
      queue_submap.pop();
    }
    queue_submap.push(*slam_transformed_scan);

    submap = mergeQueueSubmap(queue_submap);
    submap.header.frame_id = "/map";

    sensor_msgs::PointCloud2::Ptr submap_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(map_slam, *submap_ptr);
    submap_ptr->header.frame_id = "map";
    map_pub.publish(*submap_ptr);

    previous_x = ndt_pose_msg.pose.position.x;
    previous_y = ndt_pose_msg.pose.position.y;
    previous_z = ndt_pose_msg.pose.position.z;
    previous_roll = roll;
    previous_pitch = pitch;
    previous_yaw = yaw;

  }
   */

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
      count++;
      if(count <= 33){
        continue;
      }

      if(count % 10 == 0) {
        geometry_msgs::Point point;
        geometry_msgs::Quaternion orientation;

        std::string delim(" ");
        std::list <std::string> list_str;
        boost::split(list_str, line, boost::is_any_of(delim), boost::algorithm::token_compress_on);

        // Convert list to vector
        std::vector <std::string> vec_str;
        for (std::list<std::string>::iterator iter = list_str.begin(); iter != list_str.end(); iter++) {
          vec_str.push_back(*iter);
        }

        // erase first element because it is a space
        vec_str.erase(vec_str.begin());
        pospac_export pospac_data;

        pospac_data.timestamp = stringToRosTime(vec_str.at(0));
        pospac_data.timestamp.nsec *= 10000;
        ros::Duration timestamp_offset(1531007982.3);
        pospac_data.timestamp += timestamp_offset;
        pospac_data.distance = std::stod(vec_str.at(1));
        pospac_data.position.x = std::stod(vec_str.at(2));
        pospac_data.position.y = std::stod(vec_str.at(3));
        pospac_data.position.z = std::stod(vec_str.at(4));
        pospac_data.lat = std::stod(vec_str.at(5));
        pospac_data.lon = std::stod(vec_str.at(6));
        pospac_data.ellipsoid_height = std::stod(vec_str.at(7));

        double roll = std::stod(vec_str.at(8)) * M_PI / 180.0;
        double pitch = -1.0 * std::stod(vec_str.at(9)) * M_PI / 180.0;
        double yaw = -1.0 * std::stod(vec_str.at(10)) * M_PI / 180.0 + M_PI / 2.0;
        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        tf::quaternionTFToMsg(q, pospac_data.orientation);
        pospac_data.east_velocity = std::stod(vec_str.at(11));
        pospac_data.north_velocity = std::stod(vec_str.at(12));
        pospac_data.up_velocity = std::stod(vec_str.at(13));
        pospac_data.east_sd = std::stod(vec_str.at(14));
        pospac_data.north_sd = std::stod(vec_str.at(15));
        pospac_data.height_sd = std::stod(vec_str.at(16));
        pospac_data.roll_sd = std::stod(vec_str.at(17));
        pospac_data.pitch_sd = std::stod(vec_str.at(18));
        pospac_data.heading_sd = std::stod(vec_str.at(19));

        vec_pospac_export.push_back(pospac_data);

        addNewAxes(marker_array, pospac_data.position, q);
      }
    }

    marker_array_pub_array[i].publish(marker_array);
    std::cout << "Published." << std::endl;
  }

  ros::Subscriber points_sub = nh.subscribe("/points_raw", 10, points_callback);

  ros::spin();

  return 0;
}

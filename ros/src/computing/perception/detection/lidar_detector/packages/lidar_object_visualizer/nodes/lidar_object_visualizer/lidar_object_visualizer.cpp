#include "lidar_object_visualizer.h"

#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <cmath>

ObjectVisualizer::ObjectVisualizer()
{
  ros::NodeHandle private_nh_("~");
  private_nh_.param<double>("vis_id_height", vis_id_height_, 1.5);
  private_nh_.param<double>("vis_arrow_height", vis_arrow_height_, 0.5);
  private_nh_.param<bool>("vis_tracked_objects", vis_tracked_objects_, false);
  private_nh_.param<bool>("vis_velocity", vis_velocity_, true);
  private_nh_.param<bool>("is_jskbb", is_jskbb_, false);

  sub_object_array_ = node_handle_.subscribe("/detection/tracked_objects", 1, &ObjectVisualizer::callback, this);
  pub_arrow_ = node_handle_.advertise<visualization_msgs::MarkerArray>("/detection/visualize/velocity_arrow", 10);
  pub_id_    = node_handle_.advertise<visualization_msgs::MarkerArray>("/detection/visualize/target_id", 10);
  pub_jskbb_ = node_handle_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detection/visualize/jskbb", 10);
}

void ObjectVisualizer::callback(const autoware_msgs::DetectedObjectArray& input)
{
  pubRosMarkers(input);
  if(is_jskbb_)
  {
    pubJskBB(input);
  }
}

void ObjectVisualizer::pubRosMarkers(const autoware_msgs::DetectedObjectArray& input)
{
  visualization_msgs::MarkerArray marker_ids, marker_arows;

  for (size_t i = 0; i < input.objects.size(); i++)
  {
    // pose_reliable == true if tracking state is stable
    // skip vizualizing if tracking state is unstable
    if(vis_tracked_objects_)
    {
      if (!input.objects[i].pose_reliable)
      {
        continue;
      }
    }

    double velocity = input.objects[i].velocity.linear.x;

    tf::Quaternion q(input.objects[i].pose.orientation.x, input.objects[i].pose.orientation.y,
                     input.objects[i].pose.orientation.z, input.objects[i].pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // in the case motion model fit opposite direction
    if (velocity < -0.1)
    {
      velocity *= -1;
      yaw += M_PI;
      // normalize angle
      while (yaw > M_PI)
        yaw -= 2. * M_PI;
      while (yaw < -M_PI)
        yaw += 2. * M_PI;
    }

    visualization_msgs::Marker id;

    id.lifetime = ros::Duration(0.2);
    id.header.frame_id = input.header.frame_id;
    id.header.stamp = input.header.stamp;
    id.ns = "id";
    id.action = visualization_msgs::Marker::ADD;
    id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // green
    id.color.g = 1.0f;
    id.color.a = 1.0;
    id.id = input.objects[i].id;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    id.pose.position.x = input.objects[i].pose.position.x;
    id.pose.position.y = input.objects[i].pose.position.y;
    id.pose.position.z = vis_id_height_;

    // convert from RPY to quartenion
    tf::Matrix3x3 obs_mat;
    obs_mat.setEulerYPR(yaw, 0, 0);  // yaw, pitch, roll
    tf::Quaternion q_tf;
    obs_mat.getRotation(q_tf);
    id.pose.orientation.x = q_tf.getX();
    id.pose.orientation.y = q_tf.getY();
    id.pose.orientation.z = q_tf.getZ();
    id.pose.orientation.w = q_tf.getW();

    id.scale.z = 1.0;

    // not to visualize '-0.0'
    if (abs(velocity) < 0.1)
    {
      velocity = 0.0;
    }
    //converting m/s to km/h
    std::string s_velocity = std::to_string(velocity * 3.6);
    std::string modified_sv = s_velocity.substr(0, s_velocity.find(".") + 3);
    std::string text;
    if(vis_velocity_)
    {
      text = "<" + std::to_string(input.objects[i].id) + "> " + modified_sv + " km/h";
    }
    else
    {
      text = "<" + std::to_string(input.objects[i].id) + "> " ;
    }

    // std::string text = "<" + std::to_string(input.objects[i].id) + ">" + " "
    //              + std::to_string(velocity) + " m/s";
    // id.text = std::to_string(input.objects[i].id);
    id.text = text;

    marker_ids.markers.push_back(id);


    if(!vis_velocity_)
    {
      continue;
    }
    visualization_msgs::Marker arrow;
    arrow.lifetime = ros::Duration(0.2);

    // skip visualizing velocity arrow if its velocity is nearly zero
    if (abs(velocity) < 0.25)
    {
      continue;
    }

    arrow.header.frame_id = input.header.frame_id;
    arrow.header.stamp = input.header.stamp;
    arrow.ns = "arrow";
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.type = visualization_msgs::Marker::ARROW;
    // green
    arrow.color.g = 1.0f;
    arrow.color.a = 1.0;
    arrow.id = input.objects[i].id;
    // arrow.id = i;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    arrow.pose.position.x = input.objects[i].pose.position.x;
    arrow.pose.position.y = input.objects[i].pose.position.y;
    arrow.pose.position.z = vis_arrow_height_;

    arrow.pose.orientation.x = q_tf.getX();
    arrow.pose.orientation.y = q_tf.getY();
    arrow.pose.orientation.z = q_tf.getZ();
    arrow.pose.orientation.w = q_tf.getW();

    // Set the scale of the arrow -- 1x1x1 here means 1m on a side
    // arrow.scale.x = velocity;
    arrow.scale.x = 3;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;

    marker_arows.markers.push_back(arrow);
  }  // end input.objects loop
  pub_id_.publish(marker_ids);
  pub_arrow_.publish(marker_arows);
}

void ObjectVisualizer::pubJskBB(const autoware_msgs::DetectedObjectArray& input)
{
  jsk_recognition_msgs::BoundingBoxArray jskbboxes_output;
  jskbboxes_output.header = input.header;
  for (size_t i = 0; i < input.objects.size(); i++)
  {
    jsk_recognition_msgs::BoundingBox bb;
    bb.header     = input.header;
    bb.pose       = input.objects[i].pose;
    bb.dimensions = input.objects[i].dimensions;
    jskbboxes_output.boxes.push_back(bb);
  }
  pub_jskbb_.publish(jskbboxes_output);

}

#ifndef MY_MQTT_HPP
#define MY_MQTT_HPP

#include <ros/ros.h>
#include <mosquitto.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf/transform_listener.h>

#include <boost/algorithm/string.hpp>

#include <chrono>
#include <string>

#define MEASURE

class MqttSender {
public:
  MqttSender();
  ~MqttSender();
  void run(std::string address, int port, int timeout, std::string topic, int qos);
  static void on_connect(struct mosquitto *mosq, void *obj, int result);
  static void on_disconnect(struct mosquitto *mosq, void *obj, int result);
  static void on_publish(struct mosquitto *mosq, void *obj, int mid);

private:
  struct mosquitto *mqtt_client;
  std::string mqtt_address;
  int mqtt_port;
  int mqtt_timeout;
  std::string mqtt_topic;
  int mqtt_qos;

  ros::NodeHandle node_handle;
  ros::Subscriber bounding_boxes_sub;
  void bounding_boxes_cb(const jsk_recognition_msgs::BoundingBoxArray &msg);

  tf::TransformListener tf_listener;
  int transformBoundingBox(const jsk_recognition_msgs::BoundingBox *bbox, jsk_recognition_msgs::BoundingBox *bbox_transformed, const std::string target_frame, const std::string source_frame);
};

class MqttReceiver {
public:
  MqttReceiver();
  ~MqttReceiver();
  void run(std::string address, int port, int timeout, std::string topic, int qos);

private:
  static void on_connect(struct mosquitto *mosq, void *obj, int result);
  static void on_disconnect(struct mosquitto *mosq, void *obj, int result);
  static void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message);

  struct mosquitto *mqtt_client;
  std::string mqtt_address;
  int mqtt_port;
  int mqtt_timeout;
  std::string mqtt_topic;
  int mqtt_qos;

  ros::NodeHandle node_handle;
  ros::Publisher bounding_boxes_pub;

  tf::TransformListener tf_listener;
  int transformBoundingBox(const jsk_recognition_msgs::BoundingBox *bbox, jsk_recognition_msgs::BoundingBox *bbox_transformed, const std::string target_frame, const std::string source_frame);
};

#endif // MY_MQTT_HPP

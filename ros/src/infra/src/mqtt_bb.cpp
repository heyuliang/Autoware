#include "infra/mqtt_bb.hpp"

// MqttSender
MqttSender::MqttSender() : node_handle("~") {

}

MqttSender::~MqttSender() {
  mosquitto_destroy(mqtt_client);
  mosquitto_lib_cleanup();
}

void MqttSender::run(std::string address, int port, int timeout, std::string topic, int qos) {
  mqtt_address = address;
  mqtt_port = port;
  mqtt_timeout = timeout;
  mqtt_topic = topic;
  mqtt_qos = qos;

  mosquitto_lib_init();

  bool clean_session = true;
  mqtt_client = mosquitto_new(NULL, clean_session, this);
  if(mqtt_client == NULL) {
    ROS_INFO("Cannot create mosquitto object.\n");
    mosquitto_lib_cleanup();
    exit(EXIT_FAILURE);
  }

  mosquitto_connect_callback_set(mqtt_client, &MqttSender::on_connect);
  mosquitto_disconnect_callback_set(mqtt_client, &MqttSender::on_disconnect);
  mosquitto_publish_callback_set(mqtt_client, &MqttSender::on_publish);

  if(mosquitto_connect_bind(mqtt_client, mqtt_address.c_str(), mqtt_port, mqtt_timeout, NULL) != MOSQ_ERR_SUCCESS) {
    ROS_INFO("Failed to connect broker.\n");
    mosquitto_lib_cleanup();
    exit(EXIT_FAILURE);
  }

  bounding_boxes_sub = node_handle.subscribe("/bounding_boxes", 100, &MqttSender::bounding_boxes_cb, this);
}

void MqttSender::on_connect(struct mosquitto *mosq, void *obj, int result) {
  ROS_INFO("on_connect: %s(%d)\n", __FUNCTION__, __LINE__);
}

void MqttSender::on_disconnect(struct mosquitto *mosq, void *obj, int result) {
  ROS_INFO("on_disconnect: %s(%d)", __FUNCTION__, __LINE__);
}

void MqttSender::on_publish(struct mosquitto *mosq, void *obj, int mid) {

}

int MqttSender::transformBoundingBox(const jsk_recognition_msgs::BoundingBox *bbox, jsk_recognition_msgs::BoundingBox *bbox_transformed, const std::string target_frame, const std::string source_frame) {
  geometry_msgs::PoseStamped pose;
  geometry_msgs::PoseStamped pose_transformed;
  pose.header = bbox->header;
  pose.pose = bbox->pose;

  try {
    tf_listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
    tf_listener.transformPose(target_frame, ros::Time(0), pose, source_frame, pose_transformed);
  } catch(tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return 0;
  }

  bbox_transformed->header = pose.header;
  bbox_transformed->header.frame_id = target_frame;
  bbox_transformed->pose = pose_transformed.pose;
  bbox_transformed->dimensions = bbox->dimensions;
  bbox_transformed->value = bbox->value;
  bbox_transformed->label = bbox->label;

  return 1;
}

void MqttSender::bounding_boxes_cb(const jsk_recognition_msgs::BoundingBoxArray &msg) {
  #ifdef MEASURE
  std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds ns = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  std::cout << "ROS_subscribe: " << ns.count() << " ";
  #endif

  std::string send_data = "";
  for(int i = 0; i < (int)msg.boxes.size(); i++) {
    jsk_recognition_msgs::BoundingBox bbox_transformed;
    if(transformBoundingBox(&(msg.boxes[i]), &bbox_transformed, "world", "velodyne") == 0) {
      continue;
    }

    send_data += std::to_string(bbox_transformed.header.seq) + ",";
    send_data += std::to_string(bbox_transformed.header.stamp.sec) + ",";
    send_data += std::to_string(bbox_transformed.header.stamp.nsec) + ",";
    send_data += bbox_transformed.header.frame_id + ",";
    send_data += std::to_string(bbox_transformed.pose.position.x) + ",";
    send_data += std::to_string(bbox_transformed.pose.position.y) + ",";
    send_data += std::to_string(bbox_transformed.pose.position.z) + ",";
    send_data += std::to_string(bbox_transformed.pose.orientation.x) + ",";
    send_data += std::to_string(bbox_transformed.pose.orientation.y) + ",";
    send_data += std::to_string(bbox_transformed.pose.orientation.z) + ",";
    send_data += std::to_string(bbox_transformed.pose.orientation.w) + ",";
    send_data += std::to_string(bbox_transformed.dimensions.x) + ",";
    send_data += std::to_string(bbox_transformed.dimensions.y) + ",";
    send_data += std::to_string(bbox_transformed.dimensions.z) + ",";
    send_data += std::to_string(bbox_transformed.value) + ",";
    send_data += std::to_string(bbox_transformed.label) + "\n";
  }

  #ifdef MEASURE
  tp = std::chrono::high_resolution_clock::now();
  ns = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  std::cout << "MQTT_publish: " << ns.count() << std::endl;
  #endif

  mosquitto_publish(mqtt_client, NULL, mqtt_topic.c_str(), send_data.length(), send_data.c_str(), mqtt_qos, false);
}

// MqttReceiver
MqttReceiver::MqttReceiver() : node_handle("~") {

}

MqttReceiver::~MqttReceiver() {
  mosquitto_loop_stop(mqtt_client, 1);
  mosquitto_destroy(mqtt_client);
  mosquitto_lib_cleanup();
}

void MqttReceiver::run(std::string address, int port, int timeout, std::string topic, int qos) {
  mqtt_address = address;
  mqtt_port = port;
  mqtt_timeout = timeout;
  mqtt_topic = topic;
  mqtt_qos = qos;

  mosquitto_lib_init();

  bool clean_session = true;
  mqtt_client = mosquitto_new(NULL, clean_session, this);
  if(mqtt_client == NULL) {
    ROS_INFO("Cannot create mosquitto object.\n");
    mosquitto_lib_cleanup();
    exit(EXIT_FAILURE);
  }

  mosquitto_connect_callback_set(mqtt_client, &MqttReceiver::on_connect);
  mosquitto_disconnect_callback_set(mqtt_client, &MqttReceiver::on_disconnect);
  mosquitto_message_callback_set(mqtt_client, &MqttReceiver::on_message);

  if(mosquitto_connect_bind(mqtt_client, mqtt_address.c_str(), mqtt_port, mqtt_timeout, NULL) != MOSQ_ERR_SUCCESS) {
    ROS_INFO("Failed to connect broker.\n");
    mosquitto_lib_cleanup();
    exit(EXIT_FAILURE);
  }

  bounding_boxes_pub = node_handle.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bb_from_others", 10);

  mosquitto_subscribe(mqtt_client, NULL, mqtt_topic.c_str(), mqtt_qos);
  mosquitto_loop(mqtt_client, -1, 1);
  mosquitto_loop_start(mqtt_client);
}

int MqttReceiver::transformBoundingBox(const jsk_recognition_msgs::BoundingBox *bbox, jsk_recognition_msgs::BoundingBox *bbox_transformed, const std::string target_frame, const std::string source_frame) {
  geometry_msgs::PoseStamped pose;
  geometry_msgs::PoseStamped pose_transformed;
  pose.header = bbox->header;
  pose.pose = bbox->pose;

  try {
    tf_listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
    tf_listener.transformPose(target_frame, ros::Time(0), pose, source_frame, pose_transformed);
  } catch(tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return 0;
  }


  bbox_transformed->header = pose.header;
  bbox_transformed->header.frame_id = target_frame;
  bbox_transformed->pose = pose_transformed.pose;
  bbox_transformed->dimensions = bbox->dimensions;
  bbox_transformed->value = bbox->value;
  bbox_transformed->label = bbox->label;

  return 1;
}

void MqttReceiver::on_connect(struct mosquitto *mosq, void *obj, int result) {
  ROS_INFO("on_connect: %s(%d)\n", __FUNCTION__, __LINE__);
}

void MqttReceiver::on_disconnect(struct mosquitto *mosq, void *obj, int result) {
  ROS_INFO("on_disconnect: %s(%d)", __FUNCTION__, __LINE__);
}

void MqttReceiver::on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message) {
  #ifdef MEASURE
  std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds ns = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  std::cout << "MQTT_subscribe: " << ns.count() << " ";
  #endif

  MqttReceiver *node = (MqttReceiver *)obj;
  jsk_recognition_msgs::BoundingBoxArray bbox_array;

  if(message->payloadlen > 0) {
    std::string msg_str((char *)message->payload, message->payloadlen);

    std::vector<std::string> bbox_array_str;
    boost::algorithm::split(bbox_array_str, msg_str, boost::is_any_of("\n"));

    for(int i = 0; i < (int)bbox_array_str.size()-1; i++) {
      std::vector<std::string> params;
      boost::algorithm::split(params, bbox_array_str[i], boost::is_any_of(","));

      jsk_recognition_msgs::BoundingBox bbox;
      bbox.header.seq = std::stoi(params[0]);
      bbox.header.stamp.sec = std::stoi(params[1]);
      bbox.header.stamp.nsec = std::stoi(params[2]);
      bbox.header.frame_id = params[3].c_str();
      bbox.pose.position.x = std::stof(params[4]);
      bbox.pose.position.y = std::stof(params[5]);
      bbox.pose.position.z = std::stof(params[6]);
      bbox.pose.orientation.x = std::stof(params[7]);
      bbox.pose.orientation.y = std::stof(params[8]);
      bbox.pose.orientation.z = std::stof(params[9]);
      bbox.pose.orientation.w = std::stof(params[10]);
      bbox.dimensions.x = std::stof(params[11]);
      bbox.dimensions.y = std::stof(params[12]);
      bbox.dimensions.z = std::stof(params[13]);
      bbox.value = std::stof(params[14]);
      bbox.label = std::stoi(params[15]);

      jsk_recognition_msgs::BoundingBox bbox_transformed;
      if(node->transformBoundingBox(&bbox, &bbox_transformed, "velodyne", "world") == 0) {
        continue;
      }

      bbox_array.boxes.push_back(bbox_transformed);
    }

    bbox_array.header.seq = bbox_array.boxes[0].header.seq;
    bbox_array.header.stamp = bbox_array.boxes[0].header.stamp;
    bbox_array.header.frame_id = "velodyne";

    #ifdef MEASURE
    tp = std::chrono::high_resolution_clock::now();
    ns = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
    std::cout << "ROS_publish: " << ns.count() << std::endl;
    #endif

    node->bounding_boxes_pub.publish(bbox_array);
  }
}

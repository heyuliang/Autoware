#include <ros/ros.h>

#include "infra/mqtt_bb.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "mqtt_send_bb");

  MqttSender node;
  node.run("localhost", 1883, 3000, "/bb_from_others", 0);

  ros::spin();
  return 0;
}

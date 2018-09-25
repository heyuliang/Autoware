#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os, sys
import rospy
import tf
import yaml
import traceback
import paho.mqtt.client as mqtt
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from geometry_msgs.msg import PoseStamped, TwistStamped
from autoware_msgs.msg import RemoteCmd, VehicleStatus, TrafficLightResult, state

mqtt_cient = None
mqtt_config = None
pub_remote_cmd = None

def publish_mqtt_msg(topic, msg):
    try:
        if mqtt_cient is not None:
            if mqtt_config['mqtt']['TYPE'] == "MQTT":
                mqtt_cient.publish(
                    "vehicle/" + str(mqtt_config["mqtt"]["VEHICLEID"]) + "/" + topic,
                    msg
                )
            elif mqtt_config['mqtt']['TYPE'] == "AWS_IOT":
                mqtt_cient.publish(
                    "vehicle/" + str(mqtt_config["mqtt"]["VEHICLEID"]) + "/" + topic,
                    msg,
                    0
                )
    except:
        pass

def convert_csv(data_array):
    csv_msg = ""
    for data in data_array:
        csv_msg += str(data) + ","
    return csv_msg[:-1]

def vehicle_status_callback(data):
    try:
        if data.header.seq % (float(mqtt_config["mqtt"]["VEHICLESTATUS_DOWNSAMPLE"]) * 100) == 0:
            gearshift = -1
            if int(data.gearshift) == int(mqtt_config["mqtt"]["GEAR_D"]):
                gearshift = int(mqtt_config["mqtt"]["REMOTE_GEAR_D"])
            elif int(data.gearshift) == int(mqtt_config["mqtt"]["GEAR_N"]):
                gearshift = int(mqtt_config["mqtt"]["REMOTE_GEAR_N"])
            elif int(data.gearshift) == int(mqtt_config["mqtt"]["GEAR_R"]):
                gearshift = int(mqtt_config["mqtt"]["REMOTE_GEAR_R"])
            elif int(data.gearshift) == int(mqtt_config["mqtt"]["GEAR_P"]):
                gearshift = int(mqtt_config["mqtt"]["REMOTE_GEAR_P"])

            pub_data = [
                data.speed,
                data.angle,
                data.drivepedal,
                data.brakepedal,
                gearshift,
                data.lamp,
                data.light,
                data.steeringmode,
                data.drivemode
            ]
            pub_msg = convert_csv(pub_data)
            publish_mqtt_msg("vehicle_status", pub_msg)
    except:
        rospy.loginfo(traceback.format_exc())


def twist_cmd_callback(data):
    try:
        target_speed = (data.twist.linear.x * 60 * 60) / 1000 # km/h
        pub_data = [
            target_speed
        ]
        pub_msg = convert_csv(pub_data)
        publish_mqtt_msg("current_velocity", pub_msg)
    except:
        rospy.loginfo(traceback.format_exc())

def current_pose_callback(data):
    try:
        pub_data = [
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z,
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        ]
        pub_msg = convert_csv(pub_data)
        publish_mqtt_msg("current_pose", pub_msg)
    except:
        rospy.loginfo(traceback.format_exc())

def decision_maker_state_callback(data):
    try:
        pub_data = [
            data.main_state
        ]
        pub_msg = convert_csv(pub_data)
        publish_mqtt_msg("state", pub_msg)
    except:
        rospy.loginfo(traceback.format_exc())

def on_connect(client, userdata, flags, respons_code):
    rospy.loginfo("ON CONNECT TO MQTT BROKER.")
    remote_cmd_topic = "vehicle/" + str(mqtt_config['mqtt']['VEHICLEID']) + "/remote_cmd"
    client.subscribe(remote_cmd_topic)

def publish_remote_cmd(msg):
    try:
        remote_cmd = RemoteCmd()
        split_mgs = msg.split(",")

        if len(split_mgs) == 8 and pub_remote_cmd is not None:
            remote_cmd.vehicle_cmd.steer_cmd.steer = float(split_mgs[0]) * float(mqtt_config['mqtt']['STEER_MAX_VAL'])
            remote_cmd.vehicle_cmd.accel_cmd.accel = float(split_mgs[1]) * float(mqtt_config['mqtt']['ACCEL_MAX_VAL'])
            remote_cmd.vehicle_cmd.brake_cmd.brake = float(split_mgs[2]) * float(mqtt_config['mqtt']['BRAKE_MAX_VAL'])
            remote_cmd.vehicle_cmd.gear = int(split_mgs[3])
            # lamp
            remote_cmd.vehicle_cmd.lamp_cmd.l = 0
            remote_cmd.vehicle_cmd.lamp_cmd.r = 0
            if int(split_mgs[4]) == 1:
                remote_cmd.vehicle_cmd.lamp_cmd.l = 1
            elif int(split_mgs[4]) == 2:
                remote_cmd.vehicle_cmd.lamp_cmd.r = 1
            elif int(split_mgs[4]) == 3:
                remote_cmd.vehicle_cmd.lamp_cmd.l = 1
                remote_cmd.vehicle_cmd.lamp_cmd.r = 1

            remote_cmd.vehicle_cmd.twist_cmd.twist.linear.x = float(split_mgs[1]) * float(mqtt_config['mqtt']['LINEAR_X_MAX_VAL'])
            remote_cmd.vehicle_cmd.twist_cmd.twist.angular.z = float(split_mgs[0])
            remote_cmd.vehicle_cmd.ctrl_cmd.linear_velocity = float(split_mgs[1]) * float(mqtt_config['mqtt']['LINEAR_X_MAX_VAL'])
            remote_cmd.vehicle_cmd.ctrl_cmd.steering_angle = float(split_mgs[0]) * float(mqtt_config['mqtt']['STEER_MAX_VAL'])
            remote_cmd.vehicle_cmd.mode = int(split_mgs[5])
            remote_cmd.control_mode = int(split_mgs[6])
            remote_cmd.vehicle_cmd.emergency = int(split_mgs[7])

            pub_remote_cmd.publish(remote_cmd)
    except:
        rospy.loginfo(traceback.format_exc())

def on_message_for_mqtt(client, userdata, msg):
    try:
        publish_remote_cmd(msg.payload)
    except:
        rospy.loginfo(traceback.format_exc())

def on_message_for_aws_iot(client, userdata, msg):
    try:
        publish_remote_cmd(msg.payload)
    except:
        rospy.loginfo(traceback.format_exc())

if __name__ == '__main__':
    try:
        rospy.init_node("mqtt_socket")

        # ROS PUB
        pub_remote_cmd = rospy.Publisher('/remote_cmd', RemoteCmd, queue_size=1)
        # ROS SUB
        rospy.Subscriber('/vehicle_status', VehicleStatus, vehicle_status_callback)
        rospy.Subscriber('/twist_cmd', TwistStamped, twist_cmd_callback)
        rospy.Subscriber('/current_pose', PoseStamped, current_pose_callback)
        rospy.Subscriber('/decisionmaker/states', state, decision_maker_state_callback)

        config_file_path = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../mqtt_config.yml'))
        f = open(config_file_path, "r+")
        mqtt_config = yaml.load(f)
        rospy.loginfo("[MQTT BROKER] TYPE: " + mqtt_config['mqtt']['TYPE'] + ", ADDRESS: " + mqtt_config['mqtt']['ADDRESS'] + ", PORT: " + str(mqtt_config['mqtt']['PORT']))

        if mqtt_config['mqtt']['TYPE'] == "MQTT":
            mqtt_cient = mqtt.Client(client_id="vehicle_" + str(mqtt_config["mqtt"]["VEHICLEID"]), protocol=mqtt.MQTTv311)
            mqtt_cient.on_connect = on_connect
            mqtt_cient.on_message = on_message_for_mqtt
            mqtt_cient.connect(mqtt_config['mqtt']['ADDRESS'], port=mqtt_config['mqtt']['PORT'], keepalive=mqtt_config['mqtt']['TIMEOUT'])
            mqtt_cient.loop_start()

        elif mqtt_config['mqtt']['TYPE'] == "AWS_IOT":
            mqtt_cient = AWSIoTMQTTClient("vehicle_" + str(mqtt_config["mqtt"]["VEHICLEID"]))
            mqtt_cient.configureEndpoint(mqtt_config['mqtt']['ADDRESS'], int(mqtt_config['mqtt']['PORT']))
            cert_path = os.path.dirname(os.path.abspath(__file__)) + "/../cert/"
            mqtt_cient.configureCredentials(
                cert_path + mqtt_config['mqtt']['ROOTCA'],
                cert_path + mqtt_config['mqtt']['PRICATE_KEY'],
                cert_path + mqtt_config['mqtt']['CERTIFICATE']
            )
            mqtt_cient.configureAutoReconnectBackoffTime(1, 32, 20)
            mqtt_cient.configureOfflinePublishQueueing(1, dropBehavior=0)  # DROP_OLDEST = 0, DROP_NEWEST = 1
            mqtt_cient.configureDrainingFrequency(100)  # Draining: X Hz
            mqtt_cient.configureConnectDisconnectTimeout(3600)
            mqtt_cient.configureMQTTOperationTimeout(5)
            mqtt_cient.connect()

            mqtt_cient.subscribe("vehicle/" + str(mqtt_config['mqtt']['VEHICLEID']) + "/remote_cmd", 1, on_message_for_aws_iot)

        rospy.spin()
    except:
        rospy.loginfo(traceback.format_exc())

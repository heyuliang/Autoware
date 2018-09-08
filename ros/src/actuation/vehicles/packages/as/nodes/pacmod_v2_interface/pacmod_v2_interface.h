/*
 *  Copyright (c) 2017, Nagoya University
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

#ifndef PACMOD_V2_INTERFACE_H
#define PACMOD_V2_INTERFACE_H

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <pacmod_msgs/PacmodCmd.h>
#include <pacmod_msgs/PositionWithSpeed.h>




class PacmodV2Interface
{
public:
  PacmodV2Interface();

  void run();

private:
  double target_linear_velocity_;
  double target_angular_velocity_;
  double pacmod_linear_velocity_;

  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pacmod_accel_pub_;
  ros::Publisher pacmod_brake_pub_;
  ros::Publisher pacmod_steer_pub_;

  // subscriber
  ros::Subscriber autoware_twist_cmd_sub_;
  ros::Subscriber pacmod_control_mode_sub_;
  ros::Subscriber pacmod_linear_velocity_sub_;

  // ros param
  double acceleration_limit_;
  double deceleration_limit_;
  double max_angular_velocity_;
  double pid_kp_;
  double pid_ki_;
  double pid_kd_;

  // variables
  bool is_autoware_twist_cmd_initialized_;
  bool is_pacmod_control_mode_;
  bool is_pacmod_linear_velocity_initialized_;

  double current_time_ ;
  double past_time_ ;

  double integration_error_;

  // callbacks
  void callbackAutowareTwistCmd(const geometry_msgs::TwistStampedConstPtr &msg);
  void callbackPACMODControlMode(const std_msgs::BoolConstPtr &msg);
  void callbackPACMODLinearVelocity(const std_msgs::Float64ConstPtr &msg);


  double calculatePID(const double delta_velocity);
  pacmod_msgs::PacmodCmd makePACMODcmd(const double target_acceleration);
  pacmod_msgs::PositionWithSpeed makeSteerMsg(const double delta_time);

};

#endif  // PACMOD_V2_INTERFACE_H

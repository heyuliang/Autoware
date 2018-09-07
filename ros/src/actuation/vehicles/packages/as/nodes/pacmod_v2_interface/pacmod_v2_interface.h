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
  double current_linear_velocity_;
  double current_angular_velocity_;

  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher accel_pub_;
  ros::Publisher brake_pub_;
  ros::Publisher steer_pub_;

  // subscriber
  ros::Subscriber twist_cmd_sub_;
  ros::Subscriber control_mode_sub_;
  ros::Subscriber current_linear_velocity_sub_;
  ros::Subscriber current_angular_velocity_sub_;

  // ros param
  double acceleration_limit_;
  double deceleration_limit_;
  double max_angular_velocity_;
  double pid_kp_;
  double pid_ki_;
  double pid_kd_;

  // variables
  bool is_control_mode_;
  bool is_twist_cmd_initialized_;
  bool is_current_linear_velocity_initialized_;
  bool is_current_angular_velocity_initialized_;

  double current_time_ = 0;
  double past_time_ = 0;

  double integration_error_;

  // callbacks
  void callbackTwistCmd(const geometry_msgs::TwistStampedConstPtr &msg);
  void callbackPACMODControlMode(const std_msgs::BoolConstPtr &msg);
  void callbackCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg);
  void callbackEstimateTwist(const geometry_msgs::TwistStampedConstPtr &msg);


  double calculatePID(const double delta_velocity);
  pacmod_msgs::PacmodCmd makePACMODcmd(double target_acceleration);
  pacmod_msgs::PositionWithSpeed makeSteerMsg(double delta_time);


  // initializer
  // void initForROS();
};

#endif  // PACMOD_V2_INTERFACE_H

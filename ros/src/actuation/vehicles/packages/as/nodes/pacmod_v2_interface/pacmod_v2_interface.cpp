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

#include "pacmod_v2_interface.h"


// Constructor
PacmodV2Interface::PacmodV2Interface() :
    private_nh_("~"),
    is_control_mode_(true),
    is_twist_cmd_initialized_(false),
    is_current_linear_velocity_initialized_(false),
    is_current_angular_velocity_initialized_(false),
    current_time_(0),
    past_time_(0),
    integration_error_(0)
{
  // ros parameter settings
  private_nh_.param<double>("acceleration_limit", acceleration_limit_, 3.0);
  private_nh_.param<double>("deceleration_limit", deceleration_limit_, 3.0);
  private_nh_.param<double>("max_angular_velocity", max_angular_velocity_, 3.0);
  private_nh_.param<double>("pid_kp", pid_kp_, 0.9);
  private_nh_.param<double>("pid_ki", pid_ki_, 0.01);
  private_nh_.param<double>("pid_kd", pid_kd_, 0.0);

  // setup subscriber
  twist_cmd_sub_        = nh_.subscribe("/twist_cmd", 10, &PacmodV2Interface::callbackTwistCmd, this);
  current_linear_velocity_sub_ = nh_.subscribe("/as_tx/vehicle_speed", 10, &PacmodV2Interface::callbackLinearVelocity, this);
  control_mode_sub_     = nh_.subscribe("/as_tx/enable", 10, &PacmodV2Interface::callbackPACMODControlMode, this);


  // setup publisher
  accel_pub_  = nh_.advertise<pacmod_msgs::PacmodCmd>("/as_rx/accel_cmd", 10);
  brake_pub_  = nh_.advertise<pacmod_msgs::PacmodCmd>("/as_rx/brake_cmd", 10);
  steer_pub_ = nh_.advertise<pacmod_msgs::PositionWithSpeed>("/as_rx/steer_cmd", 10);
}

void PacmodV2Interface::callbackTwistCmd(const geometry_msgs::TwistStampedConstPtr &msg)
{
  target_linear_velocity_ = msg->twist.linear.x;
  target_angular_velocity_ = msg->twist.angular.z;
  is_twist_cmd_initialized_ = true;
}

void PacmodV2Interface::callbackLinearVelocity(const std_msgs::Float64ConstPtr &msg)
{
  current_linear_velocity_ = msg->data;
  is_current_linear_velocity_initialized_ = true;
}

void PacmodV2Interface::callbackPACMODControlMode(const std_msgs::BoolConstPtr &msg)
{
  is_control_mode_ = msg->data;
}

double PacmodV2Interface::calculatePID(const double velocity_error)
{
  integration_error_ += velocity_error;
  double pid_output = velocity_error*pid_kp_ + integration_error_*pid_kd_;
  return pid_output;
}

pacmod_msgs::PacmodCmd PacmodV2Interface::makePACMODcmd(double target_acceleration)
{
  double scaled_value = 0;
  if(target_acceleration > 0)
  {
    double filtered_output = std::min(target_acceleration, acceleration_limit_);
    scaled_value = filtered_output/acceleration_limit_;
    std::cout << "accel " <<  scaled_value<< std::endl;
  }
  else
  {
    double filtered_output = std::min(-1*target_acceleration, deceleration_limit_);
    scaled_value = filtered_output/deceleration_limit_;
    std::cout << "brake " <<  scaled_value<< std::endl;
  }
  double non_linear_transfromed_value = scaled_value*scaled_value;

  pacmod_msgs::PacmodCmd cmd;
  cmd.header.stamp    = ros::Time::now();
  cmd.header.frame_id = "/can";
  cmd.f64_cmd = non_linear_transfromed_value;
  cmd.enable = is_control_mode_;
  return cmd;
}

pacmod_msgs::PositionWithSpeed PacmodV2Interface::makeSteerMsg(double delta_time)
{
  pacmod_msgs::PositionWithSpeed pws;
  pws.header.stamp = ros::Time::now();
  pws.header.frame_id = "/can";
  pws.angular_position = target_angular_velocity_ * delta_time;
  pws.angular_velocity_limit = max_angular_velocity_;

  std::cout << "steer " <<  current_angular_velocity_ * delta_time<< std::endl;
  return pws;
}

void PacmodV2Interface::run()
{
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    current_time_ = ros::Time::now().toSec();
    if(is_twist_cmd_initialized_ &&
       is_current_linear_velocity_initialized_ &&
       is_current_angular_velocity_initialized_ &&
       (past_time_ != 0))
    {
      double delta_time = current_time_ - past_time_;
      double delta_velocity = target_linear_velocity_ - current_linear_velocity_;

      // double target_acceleration = calculatePID(delta_velocity);
      double target_acceleration = delta_velocity;

      pacmod_msgs::PacmodCmd cmd = makePACMODcmd(target_acceleration);
      if(target_acceleration > 0)
      {
        accel_pub_.publish(cmd);
      }
      else
      {
        brake_pub_.publish(cmd);
      }
      pacmod_msgs::PositionWithSpeed pws = makeSteerMsg(delta_time);
      steer_pub_.publish(pws);
    }
    past_time_ = ros::Time::now().toSec();
    loop_rate.sleep();
  }
}

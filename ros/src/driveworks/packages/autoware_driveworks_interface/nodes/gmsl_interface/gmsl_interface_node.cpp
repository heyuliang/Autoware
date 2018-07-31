/*
 *  Copyright (c) 2017, TierIV Inc.
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

// ROS Includes
#include <ros/ros.h>
#include <signal.h>
#include "gmsl_interface.hpp"
#include <opencv2/opencv.hpp>

volatile bool g_run = true;

void autoware_driveworks_interface::GMSLInterface::createROSMessageImage(sensor_msgs::Image &msg, const std::string &frame_id, void *rgb8_data,
                                          const ImageProperties &img_prop)
{
  uint32_t _image_width = img_prop.camera_width_ * img_prop.image_scale_;
  uint32_t _image_height = img_prop.camera_height_ * img_prop.image_scale_;

  msg.header.seq = counter_++;
  msg.header.frame_id = "camera";
  msg.header.stamp.sec = ros::Time::now().sec;
  msg.header.stamp.nsec = ros::Time::now().nsec;
  msg.height = _image_height;
  msg.width = _image_width;
  msg.encoding = img_prop.encoding_;

  msg.data.resize(_image_width * _image_height * 3);
  msg.step = _image_width * 3;

  cv::Mat current_image_rgba(cv::Size(img_prop.camera_width_, img_prop.camera_height_), CV_8UC4, (void *)rgb8_data);
  cv::resize(current_image_rgba, current_image_rgba, cv::Size(), img_prop.image_scale_, img_prop.image_scale_);

  cv::Mat current_image_rgb(cv::Size(_image_width, _image_height), CV_8UC3, (void *)msg.data.data());
  cv::cvtColor(current_image_rgba, current_image_rgb, CV_RGBA2RGB, 3);

}

void sig_int_handler(int sig)
{
  (void)sig;
  g_run = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gmsl_camera_interface");

  autoware_driveworks_interface::GMSLInterface ginterface;

  struct sigaction action;
  memset(&action, 0, sizeof(action));
  action.sa_handler = sig_int_handler;

  sigaction(SIGHUP, &action, NULL);   // controlling terminal closed, Ctrl-D
  sigaction(SIGINT, &action, NULL);   // Ctrl-C
  sigaction(SIGQUIT, &action, NULL);  // Ctrl-\, clean quit with core dump
  sigaction(SIGABRT, &action, NULL);  // abort() called.

  ginterface.init();
ginterface.setImageProp();

  ros::Rate loop_rate(20);
int counter = 0;
  while (g_run)
  {
    void *data = ginterface.getGMSLCameraImage();
   if (data != nullptr)
    {
      sensor_msgs::Image msg;
      ginterface.createROSMessageImage(msg, "camera", data, ginterface.getImageProp());
      ginterface.publishROSMessageImage(msg);
      ginterface.postProcessGMSLCameraImage(data);
    }
    loop_rate.sleep();
  }

  ginterface.releaseGMSLCamera();

  return 0;
}

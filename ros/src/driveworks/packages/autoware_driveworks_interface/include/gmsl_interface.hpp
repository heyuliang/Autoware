/*
 *  Copyright (c) 2018, TierIV Inc.
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

#ifndef GMSL_CAMERA_CORE_H
#define GMSL_CAMERA_CORE_H

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

namespace autoware_driveworks_interface
{
class ImageProperties
{
public:
  ImageProperties(){
}
  ImageProperties(uint32_t _height, uint32_t _width, double _scale, std::string _encoding)
  {
    camera_height_ = _height;
    camera_width_ = _width;
    image_scale_ = _scale;
    encoding_ = _encoding;
  }
  ~ImageProperties(){
}

  uint32_t camera_height_;
  uint32_t camera_width_;
  double image_scale_;
  std::string encoding_;
};

class DriveworksGMSLHandler
{
public:
  DriveworksGMSLHandler();

  ~DriveworksGMSLHandler();

  void run();
  int argc_;
  char **argv_;
  bool initDW();
  void postProcessGMSLCameraImage(void*);
  void releaseGMSLCamera();
  void *getGMSLCameraImage();
#define DEFAULT_SCALE 0.5
  ImageProperties getImageProp()
  {
	ImageProperties _img_prop;
	  _img_prop.camera_height_ =imageHeight;
	  _img_prop.camera_width_ = imageWidth;
	  _img_prop.image_scale_ = DEFAULT_SCALE;
	  _img_prop.encoding_ = "rgb8";
	  return _img_prop;
  }

private:
  bool gTakeScreenshot = false;
  int gScreenshotCount = 0;
  uint32_t imageWidth;
  uint32_t imageHeight;

};
class GMSLInterface
{
private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ImageProperties img_prop_;
  DriveworksGMSLHandler handle_;
  int counter_;
public:
  void init()
  {
    if (!handle_.initDW())
    {
      exit(1);
    }
    counter_ = 0;

    std::string topic(std::string("image_raw"));
    pub_ = nh_.advertise<sensor_msgs::Image>(topic, 100);
  }

  void *getGMSLCameraImage()
  {
    return handle_.getGMSLCameraImage();
  }

  void createROSMessageImage(sensor_msgs::Image &msg, const std::string &frame_id, void *rgb8_data,
                                    const ImageProperties &img_prop);
  void publishROSMessageImage(sensor_msgs::Image &msg){
	pub_.publish(msg);
  }
  void setImageProp(){
	img_prop_ = handle_.getImageProp(); 
}
  void setImageProp(const ImageProperties &prop)
  {
    img_prop_ = prop;
  }
  ImageProperties getImageProp(void)
  {
    return img_prop_;
  }
 
 void postProcessGMSLCameraImage(void *data){
	  handle_.postProcessGMSLCameraImage(data);
}
  void releaseGMSLCamera(){
	handle_.releaseGMSLCamera();
}
};
}
#endif

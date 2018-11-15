#include <string>
#include <memory>
#include <stdexcept>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "depth_completer_core.hpp"

//#include <chrono>

namespace {
  // Definition of the node name and interfaces
  const std::string kAppName = "depth_completer";
  const std::string kAppArgInputImage = "input_image_topic"; // argument name for image input. Default: "/image_cloud"
  const std::string kAppArgInputFillType = "fill_type";      // argument name for `fill_type` flag. Default: "multiscale"
  const std::string kAppArgInputExtrapolate = "extrapolate"; // argument name for `extrapolate` flag. Default: false
  const std::string kAppArgInputBlurType = "blur_type";      // argument name for 'blur_type' flag. Default: "bilateral"

  // Utility class to wrap ROS stuff
  class RosWrapper {
  public:
    //
    // Constructor
    //
    RosWrapper() :
      fill_type_("multiscale"),
      extrapolate_(false),
      blur_type_("bilateral") {
      // Get execution parameters
      GetExecutionParams();

      // Construct core class
      depth_completer_.reset(new DepthCompleter());

      // Register Subscriber
      ROS_INFO_STREAM("[" << kAppName << "] Subscribing to... " << image_topic_name_.c_str());
      image_sub_ = node_handle_.subscribe(image_topic_name_,
                                          10,
                                          &RosWrapper::ImageCallback,
                                          this
                                          );

      // Register Publisher
      image_pub_ = node_handle_.advertise<sensor_msgs::Image>("/points_depth",
                                                              10);
    }  // RosWrapper()


    //
    // Destructor
    //
    ~RosWrapper() {}

  private:
    // ROS stuff
    ros::NodeHandle node_handle_;
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
    std::string image_topic_name_;

    // Flags to control algorithm behavior
    std::string fill_type_;
    bool extrapolate_;
    std::string blur_type_;

    // The core class instance of Depth completion
    std::unique_ptr<DepthCompleter> depth_completer_;

    //
    // Function to get execution paramters
    //
    void GetExecutionParams() {
      ros::NodeHandle private_handle("~");

      private_handle.param<std::string>(kAppArgInputImage,
                                        image_topic_name_,
                                        "/image_cloud");

      private_handle.param<std::string>(kAppArgInputFillType,
                                        fill_type_,
                                        "multiscale");

      private_handle.param<bool>(kAppArgInputExtrapolate,
                                 extrapolate_,
                                 false);

      private_handle.param<std::string>(kAppArgInputBlurType,
                                        blur_type_,
                                        "bilateral");

    }  // void GetExecutionParams()


    //
    // Callback function
    //
    void ImageCallback(const sensor_msgs::Image::ConstPtr &in_image_msg) {
      cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, "mono16");
      cv::Mat input_depth_image = cv_image->image;

      //auto time_measurement_start = std::chrono::system_clock::now();

      // Load depth projections from uint16 image
      cv::Mat projected_depths;
      input_depth_image.convertTo(projected_depths, CV_32F, 1.0/255);

      // Execute completion core
      cv::Mat final_depths;
      if (fill_type_ == "fast") {
        final_depths = depth_completer_->FillInFast(projected_depths,
                                                    extrapolate_,
                                                    blur_type_);
      } else if (fill_type_ == "multiscale") {
        final_depths = depth_completer_->FillInMultiScale(projected_depths,
                                                          extrapolate_,
                                                          blur_type_);
      } else {
        std::invalid_argument("Invalid fill_type: " + fill_type_);
      }

      if (final_depths.empty()) {
        ROS_ERROR_STREAM("Empty result is deteced. Completion failed.");
        return;
      }

      cv::Mat final_image;
      final_depths.convertTo(final_image, CV_16U, 255.);

      //auto time_measurement_end = std::chrono::system_clock::now();
      //double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(time_measurement_end - time_measurement_start).count();
      //std::cerr << "fill_type: " << fill_type_ << ", elapsed: " << elapsed << "[ms]" << std::endl;

      // Pulish completion result as sensor_msgs/Image format
      cv_bridge::CvImage img_bridge = cv_bridge::CvImage(in_image_msg->header,
                                                         sensor_msgs::image_encodings::MONO16,
                                                         final_image);
      image_pub_.publish(img_bridge);
    }  // void ImageCallback()

  };  // class RosWrapper
}  // namespace


int main(int argc, char* argv[]) {
  ros::init(argc, argv, kAppName);

  RosWrapper depth_completer;

  ros::spin();

  return 0;
}

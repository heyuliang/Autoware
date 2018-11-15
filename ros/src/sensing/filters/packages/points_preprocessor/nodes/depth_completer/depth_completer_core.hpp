#ifndef DEPTH_COMPLETER_CORE_H
#define DEPTH_COMPLETER_CORE_H

#include <string>
#include <map>

#include <opencv2/opencv.hpp>

class DepthCompleter {
public:
  //
  // Constructor
  //
  DepthCompleter();


  //
  // Destructor
  //
  ~DepthCompleter() {};


  /*
    Fast, in-place depth completion.

    Args:
            depth_map: projected depths
            extrapolate: whether to extrapolate by extending depths to top of
                the frame, and applying a 31x31 full kernel dilation
            blur_type:
                'bilateral' - preserves local structure (recommended)
                'gaussian' - provides lower RMSE
            max_depth: max depth value for inversion
            custom_kernel: kernel to apply initial dilation

    Returns:
            depth_map: dense depth map
  */
  virtual cv::Mat FillInFast(const cv::Mat& depth_map,
                             const bool& extrapolate=false,
                             const std::string& blur_type="bilateral",
                             const double& max_depth=100.0,
                             const std::string& custom_kernel_str="DIAMOND_KERNEL_5");


  /*
    Slower, multi-scale dilation version with additional noise removal
    that provides better qualitative results.

    Args:
            depth_map: projected depths
            extrapolate:whether to extrapolate by extending depths to top of
                the frame, and applying a 31x31 full kernel dilation
            blur_type:
                'gaussian' - provides lower RMSE
                'bilateral' - preserves local structure (recommended)
            max_depth: max depth value for inversion
            dilation_kernel_far: dilation kernel to use for 30.0 < depths < 80.0 m
            dilation_kernel_med: dilation kernel to use for 15.0 < depths < 30.0 m
            dilation_kernel_near: dilation kernel to use for 0.1 < depths < 15.0 m

    Returns:
            depth_map: dense depth map

  */
  virtual cv::Mat FillInMultiScale(const cv::Mat& depth_map,
                                   const bool& extrapolate=false,
                                   const std::string& blur_type="bilateral",
                                   const double& max_depth=100.0,
                                   const std::string& dilation_kernel_far_str="CROSS_KERNEL_3",
                                   const std::string& dilation_kernel_med_str="CROSS_KERNEL_5",
                                   const std::string& dilation_kernel_near_str="CROSS_KERNEL_7");

private:
  /*
    Dictionary to keep dilation kernels with their name.
    Valid names are:
    - FULL_KERNEL_3
    - FULL_KERNEL_5
    - FULL_KERNEL_7
    - FULL_KERNEL_9
    - FULL_KERNEL_31
    - CROSS_KERNEL_3
    - CROSS_KERNEL_5
    - DIAMOND_KERNEL_5
    - CROSS_KERNEL_7
    - DIAMOND_KERNEL_7
  */
  std::map<std::string, cv::Mat> kernels_;
};
#endif  // DEPTH_COMPLETER_CORE_H

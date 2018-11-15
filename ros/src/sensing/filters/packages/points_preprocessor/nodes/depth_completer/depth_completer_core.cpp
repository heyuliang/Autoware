#include "depth_completer_core.hpp"
#include <stdexcept>

DepthCompleter::DepthCompleter() {
  // Initialize dilation kernel dictionary
  kernels_["FULL_KERNEL_3"]    = cv::Mat::ones(3, 3, CV_8U);
  kernels_["FULL_KERNEL_5"]    = cv::Mat::ones(5, 5, CV_8U);
  kernels_["FULL_KERNEL_7"]    = cv::Mat::ones(7, 7, CV_8U);
  kernels_["FULL_KERNEL_9"]    = cv::Mat::ones(9, 9, CV_8U);
  kernels_["FULL_KERNEL_31"]   = cv::Mat::ones(31, 31, CV_8U);
  kernels_["CROSS_KERNEL_3"]   = (cv::Mat_<uint8_t>(3, 3) <<
                                  0, 1, 0,
                                  1, 1, 1,
                                  0, 1, 0);
  kernels_["CROSS_KERNEL_5"]   = (cv::Mat_<uint8_t>(5, 5) <<
                                  0, 0, 1, 0, 0,
                                  0, 0, 1, 0, 0,
                                  1, 1, 1, 1, 1,
                                  0, 0, 1, 0, 0,
                                  0, 0, 1, 0, 0);
  kernels_["DIAMOND_KERNEL_5"] = (cv::Mat_<uint8_t>(5, 5) <<
                                  0, 0, 1, 0, 0,
                                  0, 1, 1, 1, 0,
                                  1, 1, 1, 1, 1,
                                  0, 1, 1, 1, 0,
                                  0, 0, 1, 0, 0);
  kernels_["CROSS_KERNEL_7"]   = (cv::Mat_<uint8_t>(7, 7) <<
                                  0, 0, 0, 1, 0, 0, 0,
                                  0, 0, 0, 1, 0, 0, 0,
                                  0, 0, 0, 1, 0, 0, 0,
                                  1, 1, 1, 1, 1, 1, 1,
                                  0, 0, 0, 1, 0, 0, 0,
                                  0, 0, 0, 1, 0, 0, 0,
                                  0, 0, 0, 1, 0, 0, 0);
  kernels_["DIAMOND_KERNEL_7"] = (cv::Mat_<uint8_t>(7, 7) <<
                                  0, 0, 0, 1, 0, 0, 0,
                                  0, 0, 1, 1, 1, 0, 0,
                                  0, 1, 1, 1, 1, 1, 0,
                                  1, 1, 1, 1, 1, 1, 1,
                                  0, 1, 1, 1, 1, 1, 0,
                                  0, 0, 1, 1, 1, 0, 0,
                                  0, 0, 0, 1, 0, 0, 0);
}  // DepthCompleter::DepthCompleter()


//
// Fast depth completion.
//
cv::Mat
DepthCompleter::FillInFast(const cv::Mat& depth_map,
                           const bool& extrapolate,
                           const std::string& blur_type,
                           const double& max_depth,
                           const std::string& custom_kernel_str) {
  cv::Mat depths_in = depth_map.clone();
  const int img_height = depths_in.size().height;
  const int img_width = depths_in.size().width;

  // Invert
  for (int y = 0; y < img_height; y++) {
    for (int x = 0; x < img_width; x++) {
      bool is_valid = depths_in.ptr<float>(y)[x] > 0.1;
      float current_val = depths_in.ptr<float>(y)[x];
      depths_in.ptr<float>(y)[x] = (is_valid) ? max_depth - current_val : current_val;
    }
  }

  // Dilate
  cv::dilate(depths_in, depths_in, kernels_[custom_kernel_str]);

  // Hole closing
  cv::morphologyEx(depths_in, depths_in, cv::MORPH_CLOSE, kernels_["FULL_KERNEL_5"]);

  // Fill empty spaces with dilated values
  cv::Mat dilated = depths_in.clone();
  cv::dilate(depths_in, dilated, kernels_["FULL_KERNEL_7"]);
  for (int y = 0; y < img_height; y++) {
    for (int x = 0; x < img_width; x++) {
      bool is_empty = depths_in.ptr<float>(y)[x] < 0.1;
      float current_val = depths_in.ptr<float>(y)[x];
      depths_in.ptr<float>(y)[x] = (is_empty) ? dilated.ptr<float>(y)[x] : current_val;
    }
  }

  // Extend highest pixel to top of image
  if (extrapolate) {
    for (int x = 0; x < img_width; x++) {
      float top_pixel_val = 0.0;
      int y = 0;
      while (depths_in.ptr<float>(y)[x] <= 0.1 && y < img_height) {
        top_pixel_val = depths_in.ptr<float>(y + 1)[x];
        y++;
      }

      y = 0;
      while (depths_in.ptr<float>(y)[x] <= 0.1 && y < img_height) {
        depths_in.ptr<float>(y)[x] = top_pixel_val;
        y++;
      }
    }

    // Large fill
    cv::dilate(depths_in, dilated, kernels_["FULL_KERNEL_31"]);
    for (int y = 0; y < img_height; y++) {
      for (int x = 0; x < img_width; x++) {
        bool is_empty = (depths_in.ptr<float>(y)[x] < 0.1);
        float current_val = depths_in.ptr<float>(y)[x];
        depths_in.ptr<float>(y)[x] = (is_empty) ? dilated.ptr<float>(y)[x] : current_val;
      }
    }
  }

  // Median blur
  cv::medianBlur(depths_in, depths_in, 5);

  // Bilateral or Gaussian blur
  if (blur_type == "bilateral") {
    // Bilateral blur
    cv::Mat depths_in_clone = depths_in.clone(); // As cv::bilateralFilter does not support in-place process, use clone
    cv::bilateralFilter(depths_in_clone, depths_in, 5, 1.5, 2.0);
  } else if (blur_type == "gaussian") {
    // Gaussian blur
    cv::Mat blurred = depths_in.clone();
    cv::GaussianBlur(depths_in, blurred, cv::Size(5, 5), 0);
    for (int y = 0; y < img_height; y++) {
      for (int x = 0; x < img_width; x++) {
        bool is_valid = (depths_in.ptr<float>(y)[x] > 0.1);
        float current_val = depths_in.ptr<float>(y)[x];
        depths_in.ptr<float>(y)[x] = (is_valid) ? blurred.ptr<float>(y)[x] : current_val;
      }
    }
  } else {
    std::invalid_argument("Invalid blur_type: " + blur_type);
  }

  // Invert
  for (int y = 0; y < img_height; y++) {
    for (int x = 0; x < img_width; x++) {
      bool is_valid = depths_in.ptr<float>(y)[x] > 0.1;
      float current_val = depths_in.ptr<float>(y)[x];
      depths_in.ptr<float>(y)[x] = (is_valid) ? max_depth - current_val : current_val;
    }
  }

  return depths_in;

}  // cv::Mat FillInFast()


//
// Slower, multi-scale dilation version with additional noise removal
// that provides better qualitative results.
//
cv::Mat
DepthCompleter::FillInMultiScale(const cv::Mat& depth_map,
                                 const bool& extrapolate,
                                 const std::string& blur_type,
                                 const double& max_depth,
                                 const std::string& dilation_kernel_far_str,
                                 const std::string& dilation_kernel_med_str,
                                 const std::string& dilation_kernel_near_str) {
  cv::Mat depths_in = depth_map.clone();
  const int img_height = depths_in.size().height;
  const int img_width = depths_in.size().width;

  // Calculate binary masks before inversion
  cv::Mat valid_pixels_near = cv::Mat::zeros(depth_map.size(), CV_32F);
  cv::Mat valid_pixels_med = cv::Mat::zeros(depth_map.size(), CV_32F);
  cv::Mat valid_pixels_far = cv::Mat::zeros(depth_map.size(), CV_32F);
  for (int y = 0; y < img_height; y++) {
    for (int x = 0; x < img_width; x++) {
      float pixel_value = depths_in.ptr<float>(y)[x];
      valid_pixels_near.ptr<float>(y)[x] = (pixel_value > 0.1 && pixel_value <= 15.0);
      valid_pixels_med.ptr<float>(y)[x] = (pixel_value > 15.0 && pixel_value <= 30.0);
      valid_pixels_far.ptr<float>(y)[x] = (pixel_value > 30.0);
    }
  }

  // Invert (and offset)
  cv::Mat s1_inverted_depths = depths_in.clone();
  for (int y = 0; y < img_height; y++) {
    for (int x = 0; x < img_width; x++) {
      float current_val = s1_inverted_depths.ptr<float>(y)[x];
      s1_inverted_depths.ptr<float>(y)[x] = (current_val > 0.1) ? (max_depth - current_val) : current_val;
    }
  }

  // Multi-scale dilation
  const cv::Mat dilation_kernel_far = kernels_[dilation_kernel_far_str];
  const cv::Mat dilation_kernel_med = kernels_[dilation_kernel_med_str];
  const cv::Mat dilation_kernel_near = kernels_[dilation_kernel_near_str];
  cv::Mat dilated_far = cv::Mat::zeros(depths_in.size(), CV_32F);
  cv::dilate(s1_inverted_depths.mul(valid_pixels_far),
             dilated_far,
             dilation_kernel_far);
  cv::Mat dilated_med = cv::Mat::zeros(depths_in.size(), CV_32F);
  cv::dilate(s1_inverted_depths.mul(valid_pixels_med),
             dilated_med,
             dilation_kernel_med);
  cv::Mat dilated_near = cv::Mat::zeros(depths_in.size(), CV_32F);
  cv::dilate(s1_inverted_depths.mul(valid_pixels_near),
             dilated_near,
             dilation_kernel_near);

  // Find valid pixels for each binned dilation
  for (int y = 0; y < img_height; y++) {
    for (int x = 0; x < img_width; x++) {
      valid_pixels_near.ptr<float>(y)[x] = (dilated_near.ptr<float>(y)[x] > 0.1);
      valid_pixels_med.ptr<float>(y)[x] = (dilated_med.ptr<float>(y)[x] > 0.1);
      valid_pixels_far.ptr<float>(y)[x] = (dilated_far.ptr<float>(y)[x] > 0.1);
    }
  }

  // Combine dilated versions, starting farthest to nearest
  cv::Mat s2_dilated_depths = s1_inverted_depths.clone();
  for (int y = 0; y < img_height; y++) {
    for (int x = 0; x < img_width; x++) {
      bool is_valid_far = static_cast<bool>(valid_pixels_far.ptr<float>(y)[x]);
      bool is_valid_med = static_cast<bool>(valid_pixels_med.ptr<float>(y)[x]);
      bool is_valid_near = static_cast<bool>(valid_pixels_near.ptr<float>(y)[x]);

      float current_val = s2_dilated_depths.ptr<float>(y)[x];
      s2_dilated_depths.ptr<float>(y)[x] = (is_valid_far) ? dilated_far.ptr<float>(y)[x] : current_val;

      current_val = s2_dilated_depths.ptr<float>(y)[x];
      s2_dilated_depths.ptr<float>(y)[x] = (is_valid_med) ? dilated_med.ptr<float>(y)[x] : current_val;

      current_val = s2_dilated_depths.ptr<float>(y)[x];
      s2_dilated_depths.ptr<float>(y)[x] = (is_valid_near) ? dilated_near.ptr<float>(y)[x] : current_val;
    }
  }

  // Small hole closure
  cv::Mat s3_closed_depths = s2_dilated_depths.clone();
  cv::morphologyEx(s2_dilated_depths, s3_closed_depths, cv::MORPH_CLOSE, kernels_["FULL_KERNEL_5"]);

  // Median blur to remove outliers
  cv::Mat s4_blurred_depths = s3_closed_depths.clone();
  cv::Mat blurred = s3_closed_depths.clone();
  cv::medianBlur(s3_closed_depths, blurred, 5);
  for (int y = 0; y < img_height; y++) {
    for (int x = 0; x < img_width; x++) {
      float current_val = s4_blurred_depths.ptr<float>(y)[x];
      s4_blurred_depths.ptr<float>(y)[x] = (s3_closed_depths.ptr<float>(y)[x] > 0.1) ? blurred.ptr<float>(y)[x] : current_val;
    }
  }

  // Calculate a top mask
  cv::Mat top_mask = cv::Mat::ones(depths_in.size(), CV_8U);
  for (int pixel_col_idx = 0; pixel_col_idx < img_width; pixel_col_idx++) {
    int top_pixel_row = 0;
    while (s4_blurred_depths.ptr<float>(top_pixel_row)[pixel_col_idx] <= 0.1 && top_pixel_row < img_height) {
      top_mask.ptr<bool>(top_pixel_row)[pixel_col_idx] = false;
      top_pixel_row++;
    }
  }

  // Get empty mask
  cv::Mat empty_pixels = cv::Mat::zeros(s4_blurred_depths.size(), CV_8U);
  for (int y = 0; y < img_height; y++) {
    for (int x = 0; x < img_width; x++) {
      bool is_valid_pixel = (s4_blurred_depths.ptr<float>(y)[x] > 0.1);
      empty_pixels.ptr<bool>(y)[x] = !is_valid_pixel && top_mask.ptr<bool>(y)[x];
    }
  }

  // Hole fill
  cv::Mat dilated = cv::Mat::zeros(s4_blurred_depths.size(), CV_32F);
  cv::dilate(s4_blurred_depths,
             dilated,
             kernels_["FULL_KERNEL_9"]);
  cv::Mat s5_dilated_depths = s4_blurred_depths.clone();
  for (int y = 0; y < img_height; y++) {
    for (int x = 0; x < img_width; x++) {
      float current_val = s5_dilated_depths.ptr<float>(y)[x];
      s5_dilated_depths.ptr<float>(y)[x] = (empty_pixels.ptr<bool>(y)[x]) ? dilated.ptr<float>(y)[x] : current_val;
    }
  }

  // Extend highest pixel to top of image or create top mask
  cv::Mat s6_extended_depths = s5_dilated_depths.clone();
  top_mask = cv::Mat::ones(s5_dilated_depths.size(), CV_8U);

  for (int pixel_col_idx = 0; pixel_col_idx < img_width; pixel_col_idx++) {
    if (extrapolate) {
      float top_pixel_val = 0.0;
      int y = 0;
      while (s5_dilated_depths.ptr<float>(y)[pixel_col_idx] <= 0.1 && y < img_height) {
        top_pixel_val = s5_dilated_depths.ptr<float>(y + 1)[pixel_col_idx];
        y++;
      }

      y = 0;
      while (s5_dilated_depths.ptr<float>(y)[pixel_col_idx] <= 0.1 && y < img_height) {
        s6_extended_depths.ptr<float>(y)[pixel_col_idx] = top_pixel_val;
        y++;
      }

    } else {
      // Create top mask
      int y = 0;
      while (s5_dilated_depths.ptr<float>(y)[pixel_col_idx] <= 0.1 && y < img_height) {
        top_mask.ptr<bool>(y)[pixel_col_idx] = false;
        y++;
      }
    }
  }

  // Fill large holes with masked dilations
  cv::Mat s7_blurred_depths = s6_extended_depths.clone();
  for (int i = 0; i < 6; i++) {
    cv::dilate(s7_blurred_depths, dilated, kernels_["FULL_KERNEL_5"]);

    for (int y = 0; y < img_height; y++) {
      for (int x = 0; x < img_width; x++) {
        bool is_empty = (s7_blurred_depths.ptr<float>(y)[x] < 0.1) && top_mask.ptr<bool>(y)[x];
        float current_val =  s7_blurred_depths.ptr<float>(y)[x];
        s7_blurred_depths.ptr<float>(y)[x] = (is_empty) ? dilated.ptr<float>(y)[x] : current_val;
      }
    }
  }

  // Median blur
  cv::medianBlur(s7_blurred_depths, blurred, 5);
  cv::Mat valid_pixels = cv::Mat::zeros(s7_blurred_depths.size(), CV_8U);
  for (int y = 0; y < img_height; y++) {
    for (int x = 0; x < img_width; x++) {
      bool is_valid = (s7_blurred_depths.ptr<float>(y)[x] > 0.1) && top_mask.ptr<bool>(y)[x];
      valid_pixels.ptr<bool>(y)[x] = is_valid;
      float current_val = s7_blurred_depths.ptr<float>(y)[x];
      s7_blurred_depths.ptr<float>(y)[x] = (is_valid) ? blurred.ptr<float>(y)[x] : current_val;
    }
  }

  if (blur_type == "gaussian") {
    // Gaussian blur
    cv::GaussianBlur(s7_blurred_depths, blurred, cv::Size(5, 5), 0);
    for (int y = 0; y < img_height; y++) {
      for (int x = 0; x < img_width; x++) {
        bool is_valid = (s7_blurred_depths.ptr<float>(y)[x] > 0.1) && top_mask.ptr<bool>(y)[x];
        float current_val = s7_blurred_depths.ptr<float>(y)[x];
        s7_blurred_depths.ptr<float>(y)[x] = (is_valid) ? blurred.ptr<float>(y)[x] : current_val;
      }
    }
  } else if (blur_type == "bilateral") {
    // Bilateral blur
    cv::bilateralFilter(s7_blurred_depths, blurred, 5, 0.5, 2.0);
    for (int y = 0; y < img_height; y++) {
      for (int x = 0; x < img_width; x++) {
        bool is_valid = valid_pixels.ptr<bool>(y)[x];
        float current_val = s7_blurred_depths.ptr<float>(y)[x];
        s7_blurred_depths.ptr<float>(y)[x] = (is_valid) ? blurred.ptr<float>(y)[x] : current_val;
      }
    }
  } else {
    std::invalid_argument("Invalid blur_type: " + blur_type);
  }

  // Invert (and offset)
  cv::Mat s8_inverted_depths = s7_blurred_depths.clone();
  for (int y = 0; y < img_height; y++) {
    for (int x = 0; x < img_width; x++) {
      bool is_valid = (s8_inverted_depths.ptr<float>(y)[x] > 0.1);
      float current_val = s8_inverted_depths.ptr<float>(y)[x];
      s8_inverted_depths.ptr<float>(y)[x] = (is_valid) ? max_depth - current_val : current_val;
    }
  }
  return s8_inverted_depths;

}  // cv::Mat DepthCompleter::FillInMultiScale()

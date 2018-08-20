

#include <string>
#include <opencv2/core.hpp>


cv::Mat demosaicing_CFA_Bayer_bilinear(cv::Mat src, const std::string &pattern="rggb");

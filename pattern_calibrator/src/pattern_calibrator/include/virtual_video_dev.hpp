#ifndef VIRTUAL_VIDEO_DEV_HPP_
#define VIRTUAL_VIDEO_DEV_HPP_

#include <opencv2/opencv.hpp>

class VirtualVideoDev {
 public:
  bool send_mat_to_v4l2(const cv::Mat& frame, const std::string& device_path);
};

#endif  // VIRTUAL_VIDEO_DEV_HPP_
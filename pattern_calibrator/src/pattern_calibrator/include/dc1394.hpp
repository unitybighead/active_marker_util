#ifndef DC1394_HPP_
#define DC1394_HPP_

#include <dc1394/dc1394.h>

#include <opencv2/opencv.hpp>

class DC1394 {
 public:
  DC1394();
  ~DC1394();
  bool is_available();
  cv::Mat capture();

 private:
  dc1394_t* d_;
  dc1394camera_t* camera_;

  bool is_available_ = false;
  void print_frame_info(dc1394video_frame_t* frame);
};

#endif
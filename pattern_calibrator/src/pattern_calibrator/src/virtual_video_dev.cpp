#include "virtual_video_dev.hpp"

#include <fcntl.h>

#include <iostream>
#include <string>

bool VirtualVideoDev::send_mat_to_v4l2(const cv::Mat& frame,
                                       const std::string& device_path) {
  // GStreamer pipeline for virtual device
  std::string pipeline =
      "appsrc ! videoconvert ! v4l2sink device=" + device_path;
  // Initialize VideoWriter
  cv::VideoWriter writer(pipeline,
                         0,                                // format (?)
                         30.0,                             // frame rate
                         cv::Size(frame.cols, frame.rows)  // frame size
  );

  if (!writer.isOpened()) {
    std::cerr << "Failed to open VideoWriter for " << device_path << std::endl;
    return false;
  }

  // write frame
  writer.write(frame);

  std::cout << "Frame written to " << device_path << std::endl;
  return true;
}
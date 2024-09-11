#ifndef CAMERA_GUI_HPP_
#define CAMERA_GUI_HPP_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "active_marker_msgs/msg/rgb.hpp"

namespace active_marker {

class CameraGUINode : public rclcpp::Node {
 public:
  CameraGUINode();

 private:
  using ColorMsg = active_marker_msgs::msg::RGB;

  cv::VideoCapture cap_;
  cv::Mat frame_to_display_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string yuv_text_ = "";
  std::string rgb_text_ = "";

  static void onMouse(int event, int x, int y, int, void* userdata);
  void update_frame();
};

}  // namespace active_marker

#endif  // CAMERA_GUI_HPP_

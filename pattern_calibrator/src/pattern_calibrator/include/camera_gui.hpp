#ifndef CAMERA_GUI_HPP_
#define CAMERA_GUI_HPP_

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <string>

#include "active_marker_msgs/msg/color_info.hpp"
namespace active_marker {
class CameraGUINode : public rclcpp::Node {
 public:
  CameraGUINode();

 private:
  enum class StateColor { BLUE, YELLOW, PINK, GREEN, NONE };

  using ColorInfoMsg = active_marker_msgs::msg::ColorInfo;
  using Int16Msg = std_msgs::msg::Int16;

  cv::VideoCapture cap_;
  cv::Mat frame_to_display_;
  std::string rgb_text_ = "";
  std::string yuv_text_ = "";
  StateColor state_color_ = StateColor::NONE;
  rclcpp::Publisher<ColorInfoMsg>::SharedPtr cur_color_publisher_;
  rclcpp::Publisher<ColorInfoMsg>::SharedPtr ref_color_publisher_;
  rclcpp::Publisher<Int16Msg>::SharedPtr last_key_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  static void onMouse(int event, int x, int y, int, void* userdata);
  void update_frame();
};

}  // namespace active_marker

#endif  // CAMERA_GUI_HPP_

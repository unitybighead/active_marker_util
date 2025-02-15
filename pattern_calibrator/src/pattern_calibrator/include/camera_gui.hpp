#ifndef CAMERA_GUI_HPP_
#define CAMERA_GUI_HPP_

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <string>

#include "active_marker_msgs/msg/color_info.hpp"
#include "dc1394.hpp"
namespace active_marker {
class CameraGUINode : public rclcpp::Node {
 public:
  CameraGUINode();

 private:
  enum class StateColor { BLUE, YELLOW, PINK, GREEN, NONE };

  using ColorInfoMsg = active_marker_msgs::msg::ColorInfo;
  using Int16Msg = std_msgs::msg::Int16;

  DC1394 dc1394_;
  cv::VideoCapture cap_;
  cv::Mat frame_to_display_;
  cv::Mat selected_region_;
  std::string cur_yuv_text_ = "";
  std::string cur_rgb_text_ = "";
  std::string ref_yuv_text_ = "";
  std::string ref_rgb_text_ = "";
  std::string last_key_text_ = "";
  StateColor state_color_ = StateColor::NONE;
  rclcpp::Publisher<ColorInfoMsg>::SharedPtr cur_color_publisher_;
  rclcpp::Publisher<ColorInfoMsg>::SharedPtr ref_color_publisher_;
  rclcpp::Publisher<Int16Msg>::SharedPtr last_key_publisher_;
  rclcpp::Subscription<Int16Msg>::SharedPtr last_key_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  const std::size_t update_hz_;
  bool is_zoomed_ = false;

  static void onMouse(int event, int x, int y, int flags, void* userdata);
  void set_key_state(Int16Msg::SharedPtr msg);
  void update_frame();
};

}  // namespace active_marker

#endif  // CAMERA_GUI_HPP_

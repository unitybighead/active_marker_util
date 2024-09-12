#ifndef CALIBRATOR_HPP_
#define CALIBRATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include "active_marker_msgs/msg/color_info.hpp"
#include "active_marker_msgs/msg/rgb.hpp"
#include "std_msgs/msg/bool.hpp"

namespace active_marker {
class CalibratorNode : public rclcpp::Node {
 public:
  CalibratorNode();

 private:
  using ColorInfoMsg = active_marker_msgs::msg::ColorInfo;
  using RGBMsg = active_marker_msgs::msg::RGB;
  using BoolMsg = std_msgs::msg::Bool;

  typedef struct {
    std::uint8_t r;
    std::uint8_t g;
    std::uint8_t b;
  } RGB;
  typedef struct {
    std::uint8_t y;
    std::uint8_t u;
    std::uint8_t v;
  } YUV;
  typedef struct {
    RGB rgb;
    YUV yuv;
  } color_info;

  enum class StateColor { BLUE, YELLOW, PINK, GREEN, NONE };
  const std::size_t update_hz_;
  rclcpp::Subscription<ColorInfoMsg>::SharedPtr cur_color_subscription_;
  rclcpp::Subscription<ColorInfoMsg>::SharedPtr ref_color_subscription_;
  rclcpp::Publisher<RGBMsg>::SharedPtr p_publisher_;
  rclcpp::Publisher<RGBMsg>::SharedPtr g_publisher_;
  rclcpp::Publisher<RGBMsg>::SharedPtr b_publisher_;
  rclcpp::Publisher<RGBMsg>::SharedPtr y_publisher_;
  rclcpp::Publisher<BoolMsg>::SharedPtr color_is_setting_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  StateColor state_color_ = StateColor::NONE;

  void set_cur_color(ColorInfoMsg::SharedPtr msg);
  void set_ref_color(ColorInfoMsg::SharedPtr msg);
  void update();
};
}  // namespace active_marker

#endif
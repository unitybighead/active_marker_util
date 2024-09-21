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
  } ColorInfo;

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

  ColorInfo cur_b_color_info_ = {{0, 0, 0}, {0, 0, 0}};
  ColorInfo cur_y_color_info_ = {{0, 0, 0}, {0, 0, 0}};
  ColorInfo cur_p_color_info_ = {{0, 0, 0}, {0, 0, 0}};
  ColorInfo cur_g_color_info_ = {{0, 0, 0}, {0, 0, 0}};
  ColorInfo ref_b_color_info_ = {{0, 0, 0}, {0, 0, 0}};
  ColorInfo ref_y_color_info_ = {{0, 0, 0}, {0, 0, 0}};
  ColorInfo ref_p_color_info_ = {{0, 0, 0}, {0, 0, 0}};
  ColorInfo ref_g_color_info_ = {{0, 0, 0}, {0, 0, 0}};

  RGB RGB_blue_ = {0, 0, 255};
  RGB RGB_yellow_ = {255, 255, 0};
  RGB RGB_pink_ = {255, 50, 0};
  RGB RGB_green_ = {0, 255, 0};

  void set_cur_color(ColorInfoMsg::SharedPtr msg);
  void set_ref_color(ColorInfoMsg::SharedPtr msg);
  void calibrate();
  void update();
};
}  // namespace active_marker

#endif
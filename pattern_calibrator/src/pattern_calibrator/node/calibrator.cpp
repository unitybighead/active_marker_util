#include "calibrator.hpp"

#include <chrono>
#include <memory>
#include <unordered_map>

using namespace std::chrono_literals;

namespace active_marker {
CalibratorNode::CalibratorNode()
    : Node("calibrator", "/am16"),
      update_hz_(this->declare_parameter<int>("update_hz", 10)) {
  const auto qos = rclcpp::QoS(1).best_effort();
  const auto placeholders = std::placeholders::_1;
  cur_color_subscription_ = this->create_subscription<ColorInfoMsg>(
      "cur_color", qos,
      std::bind(&CalibratorNode::set_cur_color, this, placeholders));
  ref_color_subscription_ = this->create_subscription<ColorInfoMsg>(
      "ref_color", qos,
      std::bind(&CalibratorNode::set_ref_color, this, placeholders));
  p_publisher_ = this->create_publisher<RGBMsg>("pink", qos);
  g_publisher_ = this->create_publisher<RGBMsg>("green", qos);
  b_publisher_ = this->create_publisher<RGBMsg>("blue", qos);
  y_publisher_ = this->create_publisher<RGBMsg>("yellow", qos);
  color_is_setting_publisher_ =
      this->create_publisher<BoolMsg>("color_is_setting", qos);
  timer_ = this->create_wall_timer(1000ms / update_hz_,
                                   std::bind(&CalibratorNode::update, this));
}

void CalibratorNode::set_cur_color(ColorInfoMsg::SharedPtr msg) {
  std::unordered_map<std::string, std::pair<ColorInfo*, RGB*>> color_map = {
      {"blue", {&cur_b_color_info_, &RGB_blue_}},
      {"yellow", {&cur_y_color_info_, &RGB_yellow_}},
      {"pink", {&cur_p_color_info_, &RGB_pink_}},
      {"green", {&cur_g_color_info_, &RGB_green_}}};

  auto it = color_map.find(msg->color);
  if (it != color_map.end()) {
    ColorInfo* cur_color_info = it->second.first;
    RGB* rgb_color = it->second.second;

    cur_color_info->rgb.r = msg->rgb.r;
    cur_color_info->rgb.g = msg->rgb.g;
    cur_color_info->rgb.b = msg->rgb.b;
    cur_color_info->yuv.y = msg->yuv.y;
    cur_color_info->yuv.u = msg->yuv.u;
    cur_color_info->yuv.v = msg->yuv.v;

    // calibrate
    if (msg->color == "blue") {
      if (ref_b_color_info_.yuv.y == 0) {
        RCLCPP_ERROR(this->get_logger(), "ref?");
        return;
      } else {
        if (cur_b_color_info_.yuv.y > ref_b_color_info_.yuv.y + 10) {
          rgb_color->b -= 10;
        } else if (cur_b_color_info_.yuv.y < ref_b_color_info_.yuv.y - 10) {
          rgb_color->b += 10;
        }
      }

      auto rgb_msg = RGBMsg();
      rgb_msg.r = rgb_color->r;
      rgb_msg.g = rgb_color->g;
      rgb_msg.b = rgb_color->b;
      b_publisher_->publish(rgb_msg);
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown color: %s", msg->color.c_str());
  }
}

void CalibratorNode::set_ref_color(ColorInfoMsg::SharedPtr msg) {
  std::unordered_map<std::string, ColorInfo*> color_map = {
      {"blue", &ref_b_color_info_},
      {"yellow", &ref_y_color_info_},
      {"pink", &ref_p_color_info_},
      {"green", &ref_g_color_info_}};

  auto it = color_map.find(msg->color);
  if (it != color_map.end()) {
    ColorInfo* ref_color_info = it->second;
    ref_color_info->rgb.r = msg->rgb.r;
    ref_color_info->rgb.g = msg->rgb.g;
    ref_color_info->rgb.b = msg->rgb.b;
    ref_color_info->yuv.y = msg->yuv.y;
    ref_color_info->yuv.u = msg->yuv.u;
    ref_color_info->yuv.v = msg->yuv.v;
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown color: %s", msg->color.c_str());
  }
}

void CalibratorNode::update() {
  auto bool_msg = BoolMsg();
  bool_msg.data = true;
  color_is_setting_publisher_->publish(bool_msg);
}

}  // namespace active_marker
#include "calibrator.hpp"

#include <chrono>
#include <memory>
#include <ostream>
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
  using enum StateColor;
  std::unordered_map<std::string, std::pair<ColorInfo*, StateColor>> color_map =
      {{"blue", {&cur_b_color_info_, BLUE}},
       {"yellow", {&cur_y_color_info_, YELLOW}},
       {"pink", {&cur_p_color_info_, PINK}},
       {"green", {&cur_g_color_info_, GREEN}}};

  auto itr = color_map.find(msg->color);
  if (itr != color_map.end()) {
    ColorInfo* cur_color_info = itr->second.first;
    state_color_ = itr->second.second;

    cur_color_info->rgb.r = msg->rgb.r;
    cur_color_info->rgb.g = msg->rgb.g;
    cur_color_info->rgb.b = msg->rgb.b;
    cur_color_info->yuv.y = msg->yuv.y;
    cur_color_info->yuv.u = msg->yuv.u;
    cur_color_info->yuv.v = msg->yuv.v;
    calibrate();
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

  auto itr = color_map.find(msg->color);
  if (itr != color_map.end()) {
    ColorInfo* ref_color_info = itr->second;
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

void CalibratorNode::calibrate() {
  using enum StateColor;
  std::unordered_map<StateColor, std::tuple<ColorInfo*, ColorInfo*, RGB*>>
      color_map = {
          {BLUE, {&ref_b_color_info_, &cur_b_color_info_, &RGB_blue_}},
          {YELLOW, {&ref_y_color_info_, &cur_y_color_info_, &RGB_yellow_}},
          {PINK, {&ref_p_color_info_, &cur_p_color_info_, &RGB_pink_}},
          {GREEN, {&ref_g_color_info_, &cur_g_color_info_, &RGB_green_}}};

  auto itr = color_map.find(state_color_);
  if (itr != color_map.end()) {
    if (std::get<0>(itr->second)->yuv.y == 0) {
      RCLCPP_ERROR(this->get_logger(), "ref?");
      return;
    }
    auto rgb_color = std::get<2>(itr->second);
    switch (state_color_) {
      case BLUE:
        RCLCPP_INFO(this->get_logger(), "blue");
        if (cur_b_color_info_.yuv.y > ref_b_color_info_.yuv.y + 10) {
          rgb_color->b -= 10;
        } else if (cur_b_color_info_.yuv.y < ref_b_color_info_.yuv.y - 10) {
          rgb_color->b += 10;
        }
        break;
      case YELLOW:
        RCLCPP_INFO(this->get_logger(), "yellow");
        break;
      case PINK:
        RCLCPP_INFO(this->get_logger(), "pink");
        break;
      case GREEN:
        RCLCPP_INFO(this->get_logger(), "green");
        break;
      default:
        return;
    }
    auto rgb_msg = RGBMsg();
    rgb_msg.r = rgb_color->r;
    rgb_msg.g = rgb_color->g;
    rgb_msg.b = rgb_color->b;
    b_publisher_->publish(rgb_msg);
  }
}

void CalibratorNode::update() {
  auto bool_msg = BoolMsg();
  bool_msg.data = true;
  color_is_setting_publisher_->publish(bool_msg);
}

}  // namespace active_marker
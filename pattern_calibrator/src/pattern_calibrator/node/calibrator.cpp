#include "calibrator.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <chrono>
#include <map>
#include <memory>
#include <ostream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

namespace active_marker {
CalibratorNode::CalibratorNode()
    : Node("calibrator", "/am16"),
      update_hz_(this->declare_parameter<int>("update_hz", 10)) {
  // set parameter from yaml file
  read_color_config_yaml();

  // configuration of topic
  const auto qos = rclcpp::QoS(1).best_effort();
  const auto placeholders = std::placeholders::_1;
  cur_color_subscription_ = this->create_subscription<ColorInfoMsg>(
      "cur_color", qos,
      std::bind(&CalibratorNode::set_cur_color, this, placeholders));
  ref_color_subscription_ = this->create_subscription<ColorInfoMsg>(
      "ref_color", qos,
      std::bind(&CalibratorNode::set_ref_color, this, placeholders));
  last_key_subscription_ = this->create_subscription<Int16Msg>(
      "last_key", qos,
      std::bind(&CalibratorNode::set_state, this, placeholders));
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
  std::map<std::string, std::pair<ColorInfo*, StateColor>> color_map = {
      {"blue", {&cur_b_color_info_, BLUE}},
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
  std::map<std::string, ColorInfo*> color_map = {{"blue", &ref_b_color_info_},
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

void CalibratorNode::set_state(Int16Msg::SharedPtr msg) {
  switch (msg->data) {
    using enum StateColor;
    case 'b':
    case 'B':
      state_color_ = StateColor::BLUE;
      break;
    case 'y':
    case 'Y':
      state_color_ = StateColor::YELLOW;
      break;
    case 'p':
    case 'P':
      state_color_ = StateColor::PINK;
      break;
    case 'g':
    case 'G':
      state_color_ = StateColor::GREEN;
      break;
    case 'r':
    case 'R':
      read_color_config_yaml();
      publish_all_color();
      RCLCPP_INFO(this->get_logger(), "reset color param");
      break;
    case -1:
      break;
    default:
      state_color_ = StateColor::NONE;
      break;
  }
}

void CalibratorNode::publish_all_color() {
  std::map<RGB*, rclcpp::Publisher<RGBMsg>::SharedPtr> pub_map = {
      {&RGB_blue_, b_publisher_},
      {&RGB_yellow_, y_publisher_},
      {&RGB_pink_, p_publisher_},
      {&RGB_green_, g_publisher_}};
  for (const auto& pair : pub_map) {
    RGB* val = pair.first;
    auto publisher = pair.second;
    auto msg = RGBMsg();
    msg.r = val->r;
    msg.g = val->g;
    msg.b = val->b;
    publisher->publish(msg);
  }
}

void CalibratorNode::calibrate() {
  using enum StateColor;
  std::map<StateColor, std::tuple<ColorInfo*, ColorInfo*, RGB*,
                                  rclcpp::Publisher<RGBMsg>::SharedPtr>>
      color_map = {
          {BLUE,
           {&ref_b_color_info_, &cur_b_color_info_, &RGB_blue_, b_publisher_}},
          {YELLOW,
           {&ref_y_color_info_, &cur_y_color_info_, &RGB_yellow_,
            y_publisher_}},
          {PINK,
           {&ref_p_color_info_, &cur_p_color_info_, &RGB_pink_, p_publisher_}},
          {GREEN,
           {&ref_g_color_info_, &cur_g_color_info_, &RGB_green_,
            g_publisher_}}};

  auto itr = color_map.find(state_color_);
  if (itr != color_map.end()) {
    auto ref_color_info = std::get<0>(itr->second);
    auto cur_color_info = std::get<1>(itr->second);
    auto rgb_color = std::get<2>(itr->second);
    auto publisher = std::get<3>(itr->second);

    if (ref_color_info->yuv.y == 0) {
      RCLCPP_ERROR(this->get_logger(), "ref?");
      return;
    }

    Eigen::Vector3d color_vec(rgb_color->r, rgb_color->g, rgb_color->b);
    // illuminance
    std::int16_t err = ref_color_info->yuv.y - cur_color_info->yuv.y;
    if (abs(err) > kthr_y) {
      color_vec += kgain_y * err * color_vec;
    }
    // color
    else {
      switch (state_color_) {
        case BLUE:
          break;
        case YELLOW:
          break;
        case PINK:
          break;
        case GREEN:
          break;
        default:
          return;
      }
    }
    *rgb_color = {static_cast<uint8_t>(std::clamp(color_vec[0], 0.0, 255.0)),
                  static_cast<uint8_t>(std::clamp(color_vec[1], 0.0, 255.0)),
                  static_cast<uint8_t>(std::clamp(color_vec[2], 0.0, 255.0))};
    auto rgb_msg = RGBMsg();
    rgb_msg.r = rgb_color->r;
    rgb_msg.g = rgb_color->g;
    rgb_msg.b = rgb_color->b;
    publisher->publish(rgb_msg);
  }
}

void CalibratorNode::read_color_config_yaml() {
  std::string package_path =
      ament_index_cpp::get_package_share_directory("pattern_calibrator");
  std::string config_path = package_path + "/config_path.yaml";
  try {
    YAML::Node path_node = YAML::LoadFile(config_path);
    std::string color_path = path_node["color"].as<std::string>();

    YAML::Node color_node = YAML::LoadFile(color_path);
    // std::string first_str = "/am**.ros__parameters.";
    std::map<std::string, RGB*> to_read_list = {{"blue", &RGB_blue_},
                                                {"yellow", &RGB_yellow_},
                                                {"pink", &RGB_pink_},
                                                {"green", &RGB_green_}};
    for (const auto& pair : to_read_list) {
      std::string color_name = pair.first;
      RGB* rgb_value = pair.second;
      if (color_node[color_name]) {
        rgb_value->r = color_node[color_name]["r"].as<std::uint8_t>();
        rgb_value->g = color_node[color_name]["g"].as<std::uint8_t>();
        rgb_value->b = color_node[color_name]["b"].as<std::uint8_t>();
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Error:" << e.what() << std::endl;
  }
}

void CalibratorNode::update() {
  auto bool_msg = BoolMsg();
  bool_msg.data = true;
  color_is_setting_publisher_->publish(bool_msg);
}

}  // namespace active_marker
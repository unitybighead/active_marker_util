#include "logger.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

namespace active_marker {
LoggerNode::LoggerNode()
    : Node("logger", "/am16"),
      update_hz_(this->declare_parameter<int>("update_hz", 20)) {
  const auto qos = rclcpp::QoS(1).reliable();
  const auto placeholders = std::placeholders::_1;

  p_subscription_ = this->create_subscription<ColorMsg>(
      "pink", qos, std::bind(&LoggerNode::set_pink, this, placeholders));
  g_subscription_ = this->create_subscription<ColorMsg>(
      "green", qos, std::bind(&LoggerNode::set_green, this, placeholders));
  b_subscription_ = this->create_subscription<ColorMsg>(
      "blue", qos, std::bind(&LoggerNode::set_blue, this, placeholders));
  y_subscription_ = this->create_subscription<ColorMsg>(
      "yellow", qos, std::bind(&LoggerNode::set_yellow, this, placeholders));
  illuminance_subscription_ = this->create_subscription<IlluminanceMsg>(
      "illuminance", rclcpp::QoS(1).best_effort(),
      std::bind(&LoggerNode::set_illuminance, this, placeholders));
  last_key_subscription_ = this->create_subscription<Int16Msg>(
      "last_key", qos, std::bind(&LoggerNode::check_key, this, placeholders));
  timer_ = this->create_wall_timer(1000ms / update_hz_,
                                   std::bind(&LoggerNode::update, this));
  no_recv_count_ = 0;
}

void LoggerNode::set_blue(ColorMsg::SharedPtr color_msg) {
  blue_.r = color_msg->r;
  blue_.g = color_msg->g;
  blue_.b = color_msg->b;
}

void LoggerNode::set_yellow(ColorMsg::SharedPtr color_msg) {
  yellow_.r = color_msg->r;
  yellow_.g = color_msg->g;
  yellow_.b = color_msg->b;
}

void LoggerNode::set_pink(ColorMsg::SharedPtr color_msg) {
  pink_.r = color_msg->r;
  pink_.g = color_msg->g;
  pink_.b = color_msg->b;
}

void LoggerNode::set_green(ColorMsg::SharedPtr color_msg) {
  green_.r = color_msg->r;
  green_.g = color_msg->g;
  green_.b = color_msg->b;
}

void LoggerNode::set_illuminance(IlluminanceMsg::SharedPtr illuminance_msg) {
  illuminance_ = (uint16_t)illuminance_msg->data;
  no_recv_count_ = 0;
}

void LoggerNode::check_key(Int16Msg::SharedPtr msg) {
  switch (msg->data) {
    case 'l':
    case 'L':
      write_config_csv();
      break;
    default:
      break;
  }
}

void LoggerNode::write_config_csv() {
  if (no_recv_count_ > 2 * update_hz_) {
    RCLCPP_ERROR(this->get_logger(), "No receive illuminance topic");
    return;
  }

  const std::string package_path =
      ament_index_cpp::get_package_share_directory("pattern_calibrator");
  const std::string config_path = package_path + "/config_path.yaml";
  try {
    const YAML::Node path_node = YAML::LoadFile(config_path);
    const std::string log_path = path_node["log"].as<std::string>();

    std::ofstream file(log_path, std::ios::app);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s",
                   log_path.c_str());
      return;
    }

    file << static_cast<int>(illuminance_) << "," << static_cast<int>(blue_.r)
         << "," << static_cast<int>(blue_.g) << "," << static_cast<int>(blue_.b)
         << "," << static_cast<int>(yellow_.r) << ","
         << static_cast<int>(yellow_.g) << "," << static_cast<int>(yellow_.b)
         << "," << static_cast<int>(pink_.r) << "," << static_cast<int>(pink_.g)
         << "," << static_cast<int>(pink_.b) << ","
         << static_cast<int>(green_.r) << "," << static_cast<int>(green_.g)
         << "," << static_cast<int>(green_.b) << "\n";

    file.close();

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during CSV write: %s",
                 e.what());
  }
}

void LoggerNode::update() { no_recv_count_++; }

}  // namespace active_marker
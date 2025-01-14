#include "logger.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

namespace active_marker {
LoggerNode::LoggerNode()
    : Node("logger", "/am16"),
      update_hz_(this->declare_parameter<int>("update_hz", 30)),
      ID_(this->declare_parameter<int>("ID", 16)),
      team_color_(this->declare_parameter<std::string>("team_color", "blue")) {
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
  robot_info_client_ = this->create_client<RobotInfoSrv>("robot_info");
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

  PosRelation_t robot_camera_relation = {0, 0};
  if (is_robot_info_received_) {
    RobotInfo_t robot_info = received_robot_info_;
    RCLCPP_INFO(this->get_logger(), "%f %f %f", robot_info.x, robot_info.y,
                robot_info.height);
    Eigen::Vector3d robot_pos = {robot_info.x, robot_info.y, robot_info.height};
    Eigen::Vector3d camera_pos = {0, 0, 0};
    try {
      const YAML::Node path_node = YAML::LoadFile(config_path);
      const std::string camera_path = path_node["camera"].as<std::string>();

      YAML::Node camera_node = YAML::LoadFile(camera_path);
      camera_pos.x() = camera_node["x"].as<double>();
      camera_pos.y() = camera_node["y"].as<double>();
      camera_pos.z() = camera_node["z"].as<double>();
    } catch (std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load camera position");
    }
    if (robot_pos.norm() != 0 && camera_pos.norm() != 0) {
      robot_camera_relation = calc_pos_relation(robot_pos, camera_pos);
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "No receive service");
  }

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
         << "," << static_cast<int>(green_.b);

    if (robot_camera_relation.distance != 0) {
      file << "," << robot_camera_relation.distance << ","
           << robot_camera_relation.elev_ang;
    }
    file << "\n";
    file.close();

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during CSV write: %s",
                 e.what());
  }
}

void LoggerNode::request_robot_info(int& ID, std::string& team_color) {
  auto request = std::make_shared<RobotInfoSrv::Request>();
  request->id = ID;
  request->color = team_color;

  auto result = robot_info_client_->async_send_request(
      request,
      std::bind(&LoggerNode::robot_info_callback, this, std::placeholders::_1));
  is_robot_info_received_ = false;
}

void LoggerNode::robot_info_callback(
    rclcpp::Client<RobotInfoSrv>::SharedFuture future) {
  try {
    auto response = future.get();
    received_robot_info_.received = response->received;
    received_robot_info_.x = response->x;
    received_robot_info_.y = response->y;
    received_robot_info_.height = response->height;
    is_robot_info_received_ = true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Exception while processing robot_info response: %s",
                 e.what());
  }
}

LoggerNode::PosRelation_t LoggerNode::calc_pos_relation(Eigen::Vector3d& pos1,
                                                        Eigen::Vector3d& pos2) {
  constexpr double PI = acos(-1);
  PosRelation_t retval;
  Eigen::Vector3d diff = pos2 - pos1;
  double vertical_diff = pos2.z() - pos1.z();
  retval.distance = diff.norm();
  if (retval.distance != 0) {
    retval.elev_ang =
        abs(std::asin(vertical_diff / retval.distance) / PI * 180);
  } else {
    retval.elev_ang = 0;
  }

  return retval;
}

void LoggerNode::update() {
  no_recv_count_++;
  static int i = 0;
  i++;
  if (i % 5 == 0) {
    i = 0;
    if (i % 5 == 0 && is_service_available(robot_info_client_)) {
      request_robot_info(ID_, team_color_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Service is not available");
    }
  }
}

}  // namespace active_marker
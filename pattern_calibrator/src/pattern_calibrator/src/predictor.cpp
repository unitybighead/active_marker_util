#include "predictor.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <map>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace active_marker {
using namespace std::chrono_literals;
PredictorNode::PredictorNode()
    : Node("predictor", "/am16"),
      update_hz_(this->declare_parameter<int>("update_hz", 10)),
      ID_(this->declare_parameter<int>("ID", 16)),
      team_color_(this->declare_parameter<std::string>("team_color", "blue")),
      camera_height_(this->declare_parameter<double>("camera_height", 2500)) {
  const auto qos = rclcpp::QoS(1).reliable();
  const auto placeholders = std::placeholders::_1;

  p_publisher_ = this->create_publisher<RGBMsg>("pink", qos);
  g_publisher_ = this->create_publisher<RGBMsg>("green", qos);
  b_publisher_ = this->create_publisher<RGBMsg>("blue", qos);
  y_publisher_ = this->create_publisher<RGBMsg>("yellow", qos);
  illuminance_subscription_ = this->create_subscription<IlluminanceMsg>(
      "illuminance", rclcpp::QoS(1).best_effort(),
      std::bind(&PredictorNode::set_illuminance, this, placeholders));
  robot_info_client_ = this->create_client<RobotInfoSrv>("robot_info");
  timer_ = this->create_wall_timer(1000ms / update_hz_,
                                   std::bind(&PredictorNode::update, this));
  no_recv_count_ = 0;
  RCLCPP_INFO(this->get_logger(), "initialized");

  // read coef of predictor
  const std::string package_path =
      ament_index_cpp::get_package_share_directory("pattern_calibrator");
  const std::string config_path = package_path + "/config/config_path.yaml";
  try {
    const YAML::Node path_node = YAML::LoadFile(config_path);
    const std::string coef_path = path_node["reg_coef"].as<std::string>();

    YAML::Node data_node = YAML::LoadFile(coef_path);

    const std::string pattern_colors[] = {"blue", "yellow", "pink", "green"};
    PredEq_t* to_read_list[] = {&blue_coef_, &yellow_coef_, &pink_coef_,
                                &green_coef_};

    for (int i = 0; i < 4; i++) {
      to_read_list[i]->r = {
          data_node[pattern_colors[i]]["red"]["b"].as<float>(),
          data_node[pattern_colors[i]]["red"]["i"].as<float>(),
          data_node[pattern_colors[i]]["red"]["d"].as<float>(),
          data_node[pattern_colors[i]]["red"]["h"].as<float>(),
      };
      to_read_list[i]->g = {
          data_node[pattern_colors[i]]["green"]["b"].as<float>(),
          data_node[pattern_colors[i]]["green"]["i"].as<float>(),
          data_node[pattern_colors[i]]["green"]["d"].as<float>(),
          data_node[pattern_colors[i]]["green"]["h"].as<float>(),
      };
      to_read_list[i]->b = {
          data_node[pattern_colors[i]]["blue"]["b"].as<float>(),
          data_node[pattern_colors[i]]["blue"]["i"].as<float>(),
          data_node[pattern_colors[i]]["blue"]["d"].as<float>(),
          data_node[pattern_colors[i]]["blue"]["h"].as<float>(),
      };
    }
  } catch (std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load coefficient");
  }
}

void PredictorNode::set_illuminance(IlluminanceMsg::SharedPtr illuminance_msg) {
  illuminance_ = (uint16_t)illuminance_msg->data;
  no_recv_count_ = 0;
}

void PredictorNode::publish_color() {
  const std::string pattern_colors[] = {"blue", "yellow", "pink", "green"};
  rclcpp::Publisher<RGBMsg>::SharedPtr publishers[] = {
      b_publisher_, y_publisher_, p_publisher_, g_publisher_};
  for (int i = 0; i < 4; i++) {
    auto msg = RGBMsg();
    double min = 0.0;
    double max = 255.0;
    msg.r =
        uint8_t(std::clamp(predict_output(pattern_colors[i], "red"), min, max));
    msg.g = uint8_t(
        std::clamp(predict_output(pattern_colors[i], "green"), min, max));
    msg.b = uint8_t(
        std::clamp(predict_output(pattern_colors[i], "blue"), min, max));
    // RCLCPP_INFO(this->get_logger(), "%s r:%d g:%d b:%d",
    // pattern_colors[i].c_str(), msg.r, msg.g, msg.b);
    publishers[i]->publish(msg);
  }
}

double PredictorNode::predict_output(std::string color,
                                     std::string composition) {
  // Japanese traditional GOMI program
  static std::map<std::string, PredEq_t*> eq_list = {{"blue", &blue_coef_},
                                                     {"yellow", &yellow_coef_},
                                                     {"pink", &pink_coef_},
                                                     {"green", &green_coef_}};

  PredEq_t* pred_eq;

  // Check if the color exists in the map
  auto it = eq_list.find(color);
  if (it != eq_list.end()) {
    pred_eq = it->second;
  } else {
    throw std::invalid_argument("Invalid color provided.");
  }

  PredCoef_t pred_coef;
  if (composition == "red") {
    pred_coef = pred_eq->r;
  } else if (composition == "green") {
    pred_coef = pred_eq->g;
  } else if (composition == "blue") {
    pred_coef = pred_eq->b;
  } else {
    throw std::invalid_argument("Invalid composition provided.");
  }

  double x = received_robot_info_.x;
  double y = received_robot_info_.y;
  double z = camera_height_;
  double distance = std::sqrt(x * x + y * y + z * z);
  double output = float(pred_coef.b + pred_coef.i * float(illuminance_) +
                        pred_coef.d * distance + pred_coef.h * z);
  RCLCPP_INFO(this->get_logger(), "coef %f %f %f %f", pred_coef.b, pred_coef.i,
              pred_coef.d, pred_coef.h);
  RCLCPP_INFO(this->get_logger(), "%d %f %f", illuminance_, distance, z);
  RCLCPP_INFO(this->get_logger(), "%s %s %f", color.c_str(),
              composition.c_str(), output);
  return output;
}

void PredictorNode::request_robot_info(int& ID, std::string& team_color) {
  auto request = std::make_shared<RobotInfoSrv::Request>();
  request->id = ID;
  request->color = team_color;

  auto result = robot_info_client_->async_send_request(
      request, std::bind(&PredictorNode::robot_info_callback, this,
                         std::placeholders::_1));
  is_robot_info_received_ = false;
}

void PredictorNode::robot_info_callback(
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

void PredictorNode::update() {
  if (is_service_available(robot_info_client_)) {
    request_robot_info(ID_, team_color_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Service is not available");
  }
  publish_color();
}
}  // namespace active_marker
#include "all_publisher.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <map>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace active_marker {
AllPublisherNode::AllPublisherNode() : Node("all_publisher", "/gui") {
  const auto qos = rclcpp::QoS(1).reliable();
  static std::map<std::vector<rclcpp::Publisher<RGBMsg>::SharedPtr>*,
                  std::string>
      publishers_list = {{&publishers_blue_, "blue"},
                         {&publishers_yellow_, "yellow"},
                         {&publishers_pink_, "pink"},
                         {&publishers_green_, "green"}};
  for (auto& pair : publishers_list) {
    auto* publishers = pair.first;
    std::string color = pair.second;
    for (int i = 0; i <= 16; ++i) {
      std::string topic_name = "/am" + std::to_string(i) + "/" + color;
      auto publisher = this->create_publisher<RGBMsg>(topic_name, qos);
      publishers->push_back(publisher);
    }
  }

  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&AllPublisherNode::publish_all, this));
}

void AllPublisherNode::publish_all() {
  read_config();
  const static std::map<std::vector<rclcpp::Publisher<RGBMsg>::SharedPtr>*,
                        RGB*>
      pub_map = {
          {&publishers_blue_, &RGB_blue_},
          {&publishers_yellow_, &RGB_yellow_},
          {&publishers_pink_, &RGB_pink_},
          {&publishers_green_, &RGB_green_},
      };
  for (const auto& pair : pub_map) {
    auto* publishers = pair.first;
    RGB* val = pair.second;
    auto msg = RGBMsg();
    msg.r = val->r;
    msg.g = val->g;
    msg.b = val->b;

    for (auto& publisher : *publishers) {
      // Publish only if subscriver is available.
      if (publisher->get_subscription_count() > 0) {
        publisher->publish(msg);
      }
    }
  }
  RCLCPP_INFO(this->get_logger(), "published to all robots.");
}

void AllPublisherNode::read_config() {
  std::string package_path =
      ament_index_cpp::get_package_share_directory("all_publisher");
  std::string config_path = package_path + "/config_path.yaml";
  try {
    YAML::Node path_node = YAML::LoadFile(config_path);
    std::string color_path = path_node["result"].as<std::string>();

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

}  // namespace active_marker

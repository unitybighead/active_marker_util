#include "vision_receiver.hpp"

#include <chrono>

using namespace std::chrono_literals;
namespace active_marker {
VisionReceiverNode::VisionReceiverNode()
    : Node("vision_receiver", "/am16"),
      update_hz_(this->declare_parameter<int>("update_hz", 10)),
      udp_(this->declare_parameter<int>("vision_port", 10006),
           this->declare_parameter<std::string>("multicast_address",
                                                "224.5.23.2"),
           std::bind(&VisionReceiverNode::parse_packet, this,
                     std::placeholders::_1)) {
  robot_info_service_ = this->create_service<RobotInfoSrv>(
      "robot_info", std::bind(&VisionReceiverNode::serve_robot_info, this,
                              std::placeholders::_1, std::placeholders::_2));
  timer_ = this->create_wall_timer(
      1000ms / update_hz_, std::bind(&VisionReceiverNode::update, this));
}

void VisionReceiverNode::parse_packet(std::vector<uint8_t> data) {
  std::string data_string(data.begin(), data.end());

  if (!packet_.ParseFromArray(data.data(), data.size())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse packet.");
    return;
  }
  no_recv_count_ = 0;
  is_received_ = true;
}

bool VisionReceiverNode::read_robot_location(
    ssl_protos::vision::Packet& packet,
    std::vector<RobotInfo_t>& robots_info_blue,
    std::vector<RobotInfo_t>& robots_info_yellow) {
  if (!packet.has_detection()) {
    return false;
  }
  std::vector<ssl_protos::vision::Robot> robots_blue(
      packet.detection().robots_blue().begin(),
      packet.detection().robots_blue().end());
  std::vector<ssl_protos::vision::Robot> robots_yellow(
      packet.detection().robots_yellow().begin(),
      packet.detection().robots_yellow().end());

  double t_capture = packet.detection().t_capture();
  last_t_capture_ = t_capture;

  for (const auto& robot : robots_blue) {
    uint32_t robot_id = robot.robot_id();
    robots_info_blue[robot_id] = {robot.x(), robot.y(), robot.height(),
                                  t_capture};
  }
  for (const auto& robot : robots_yellow) {
    uint32_t robot_id = robot.robot_id();
    robots_info_yellow[robot_id] = {robot.x(), robot.y(), robot.height(),
                                    t_capture};
  }
  return true;
}

void VisionReceiverNode::serve_robot_info(
    const std::shared_ptr<RobotInfoSrv::Request> request,
    const std::shared_ptr<RobotInfoSrv::Response> response) {
  uint32_t ID = request->id;
  std::string color = request->color;
  std::vector<RobotInfo_t>* robots_info;
  if (color == "blue") {
    robots_info = &robots_info_blue_;
  } else if (color == "yellow") {
    robots_info = &robots_info_yellow_;
  } else {
    RCLCPP_ERROR(this->get_logger(), "invalid team color");
    response->x = 0;
    response->y = 0;
    response->height = 0;
    response->received = false;
    return;
  }
  response->x = robots_info->at(ID).x;
  response->y = robots_info->at(ID).y;
  response->height = robots_info->at(ID).height;

  // TODO: processing for caputuring error
  response->received =
      (is_received_ && last_t_capture_ - robots_info->at(ID).t_capture < 1);
}

bool VisionReceiverNode::is_received() { return is_received_; }

void VisionReceiverNode::update() {
  if (no_recv_count_ > update_hz_) {
    is_received_ = false;
    return;
  }
  if (is_received_) {
    if (!read_robot_location(packet_, robots_info_blue_, robots_info_yellow_)) {
      RCLCPP_ERROR(this->get_logger(), "packet does not have robots");
    }
  }
  no_recv_count_++;
}
}  // namespace active_marker
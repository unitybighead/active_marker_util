#include "vision_receiver.hpp"

#include <chrono>

#include "ssl-protos/vision_wrapper_tracked.pb.h"

using namespace std::chrono_literals;
namespace active_marker {
VisionReceiverNode::VisionReceiverNode()
    : Node("vision_receiver", "/am16"),
      update_hz_(this->declare_parameter<int>("update_hz", 60)),
      udp_(this->declare_parameter<int>("vision_port", 10006),
           this->declare_parameter<std::string>("multicast_address",
                                                "224.5.23.2"),
           std::bind(&VisionReceiverNode::serialize_packet, this,
                     std::placeholders::_1)) {
  timer_ = this->create_wall_timer(
      1000ms / update_hz_, std::bind(&VisionReceiverNode::update, this));
}

void VisionReceiverNode::serialize_packet(std::vector<uint8_t> data) {
  ssl_protos::vision::TrackerWrapperPacket packet;
  std::string data_string(data.begin(), data.end());

  if (!packet.ParseFromArray(data.data(), data.size())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse packet.");
    return;
  }
}

void VisionReceiverNode::update() {}
}  // namespace active_marker
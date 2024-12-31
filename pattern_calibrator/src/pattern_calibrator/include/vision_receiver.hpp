#ifndef VISION_RECEIVER_HPP_
#define VISION_RECEIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "multicast.hpp"

namespace active_marker {
class VisionReceiverNode : public rclcpp::Node {
 public:
  VisionReceiverNode();

 private:
  std::size_t update_hz_;
  rclcpp::TimerBase::SharedPtr timer_;

  UdpReceiver udp_;

  void serialize_packet(std::vector<uint8_t> data);
  void update();
};
}  // namespace active_marker

#endif
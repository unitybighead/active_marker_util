#ifndef VISION_RECEIVER_HPP_
#define VISION_RECEIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "active_marker_msgs/srv/robot_info.hpp"
#include "multicast.hpp"
#include "ssl-protos/vision_wrapper.pb.h"

namespace active_marker {
class VisionReceiverNode : public rclcpp::Node {
 public:
  struct RobotInfo_t {
    float x;
    float y;
    float height;
    double t_capture;
  };

  VisionReceiverNode();

 private:
  using RobotInfoSrv = active_marker_msgs::srv::RobotInfo;

  std::size_t update_hz_;
  std::size_t no_recv_count_ = 0;

  rclcpp::Service<RobotInfoSrv>::SharedPtr robot_info_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  UdpReceiver udp_;
  ssl_protos::vision::Packet packet_;

  bool is_received_ = false;
  double last_t_capture_ = 0;
  std::vector<RobotInfo_t> robots_info_blue_{16, {0.0f, 0.0f, 0.0f, 0.0}};
  std::vector<RobotInfo_t> robots_info_yellow_{16, {0.0f, 0.0f, 0.0f, 0.0}};

  void parse_packet(std::vector<uint8_t> data);
  bool read_robot_location(ssl_protos::vision::Packet& packet,
                           std::vector<RobotInfo_t>& robots_info_blue,
                           std::vector<RobotInfo_t>& robots_info_yellow);
  void serve_robot_info(const std::shared_ptr<RobotInfoSrv::Request> request,
                        const std::shared_ptr<RobotInfoSrv::Response> response);
  bool is_received();
  bool is_received(int ID);

  void update();
};
}  // namespace active_marker

#endif
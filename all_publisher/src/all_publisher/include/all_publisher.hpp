#ifndef ALL_PUBLISH_HPP_
#define ALL_PUBLISH_HPP_

#include <rclcpp/rclcpp.hpp>

#include "active_marker_msgs/msg/rgb.hpp"

namespace active_marker {
class AllPublisherNode : public rclcpp::Node {
 public:
  typedef struct {
    std::uint8_t r;
    std::uint8_t g;
    std::uint8_t b;
  } RGB;
  typedef struct {
    std::uint8_t y;
    std::uint8_t u;
    std::uint8_t v;
  } YUV;

  RGB RGB_blue_ = {0, 0, 255};
  RGB RGB_yellow_ = {255, 255, 0};
  RGB RGB_pink_ = {255, 0, 50};
  RGB RGB_green_ = {0, 255, 0};

  AllPublisherNode();

 private:
  using RGBMsg = active_marker_msgs::msg::RGB;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Publisher<RGBMsg>::SharedPtr> publishers_blue_;
  std::vector<rclcpp::Publisher<RGBMsg>::SharedPtr> publishers_yellow_;
  std::vector<rclcpp::Publisher<RGBMsg>::SharedPtr> publishers_pink_;
  std::vector<rclcpp::Publisher<RGBMsg>::SharedPtr> publishers_green_;

  void publish_all();
  void read_config();
};
}  // namespace active_marker

#endif
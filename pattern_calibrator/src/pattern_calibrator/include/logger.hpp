#ifndef LOGGER_HPP_
#define LOGGER_HPP_
#include <rclcpp/rclcpp.hpp>

#include "active_marker_msgs/msg/rgb.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int16.hpp"

namespace active_marker {
class LoggerNode : public rclcpp::Node {
 public:
  typedef struct {
    std::uint8_t r;
    std::uint8_t g;
    std::uint8_t b;
  } RGB;

  LoggerNode();

 private:
  using ColorMsg = active_marker_msgs::msg::RGB;
  using IlluminanceMsg = std_msgs::msg::UInt16;
  using Int16Msg = std_msgs::msg::Int16;

  const std::size_t update_hz_;

  rclcpp::Subscription<ColorMsg>::SharedPtr p_subscription_;
  rclcpp::Subscription<ColorMsg>::SharedPtr g_subscription_;
  rclcpp::Subscription<ColorMsg>::SharedPtr b_subscription_;
  rclcpp::Subscription<ColorMsg>::SharedPtr y_subscription_;
  rclcpp::Subscription<IlluminanceMsg>::SharedPtr illuminance_subscription_;
  rclcpp::Subscription<Int16Msg>::SharedPtr last_key_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  RGB blue_, yellow_, pink_, green_;
  uint16_t illuminance_;
  std::size_t no_recv_count_;

  void set_pink(ColorMsg::SharedPtr color_msg);
  void set_green(ColorMsg::SharedPtr color_msg);
  void set_blue(ColorMsg::SharedPtr color_msg);
  void set_yellow(ColorMsg::SharedPtr color_msg);
  void set_illuminance(IlluminanceMsg::SharedPtr illuminance_msg);
  void check_key(Int16Msg::SharedPtr msg);
  void write_config_csv();
  void update();
};
}  // namespace active_marker

#endif
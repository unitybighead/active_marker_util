#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <string>

#include "dc1394.hpp"
namespace active_marker {
class LoggerNode : public rclcpp::Node {
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

  LoggerNode();

 private:
  using IlluminanceMsg = std_msgs::msg::UInt16;
  enum class StateColor { BLUE, YELLOW, PINK, GREEN, NONE };

  DC1394 dc1394_;
  cv::VideoCapture cap_;
  cv::Mat frame_to_display_;
  cv::Mat selected_region_;
  std::string yuv_text_ = "";
  std::string rgb_text_ = "";
  StateColor state_color_ = StateColor::NONE;
  rclcpp::Subscription<IlluminanceMsg>::SharedPtr illuminance_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  static constexpr size_t update_hz_ = 60;

  std::string color_csv_ = "";
  bool is_zoomed_ = false;
  std::uint16_t illuminance_ = 0;
  size_t no_recv_count_ = 0;

  static void onMouse(int event, int x, int y, int flags, void* userdata);
  void set_illuminance(IlluminanceMsg::SharedPtr msg);
  void write_parameter_csv(const std::string color, const RGB rgb,
                           const YUV yuv, const std::string filename);
  void update_frame();
};

}  // namespace active_marker

#endif  // LOGGER_HPP_

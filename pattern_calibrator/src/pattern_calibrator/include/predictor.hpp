#ifndef PREDICTOR_HPP_
#define PREDICTOR_HPP_
#include <rclcpp/rclcpp.hpp>

#include "active_marker_msgs/msg/rgb.hpp"
#include "active_marker_msgs/srv/robot_info.hpp"
#include "logger.hpp"

namespace active_marker {
class PredictorNode : public rclcpp::Node {
 public:
  struct RobotInfo_t {
    bool received;
    double x;
    double y;
    double height;
  };

  struct PredCoef_t {
    float b;  // intercept
    float i;  // illuminance
    float d;  // distance
    float h;  // camera_height
  };

  struct PredEq_t {
    PredCoef_t r;
    PredCoef_t g;
    PredCoef_t b;
  };

  PredictorNode();

 private:
  using RGBMsg = active_marker_msgs::msg::RGB;
  using RobotInfoSrv = active_marker_msgs::srv::RobotInfo;
  using IlluminanceMsg = std_msgs::msg::UInt16;

  rclcpp::Publisher<RGBMsg>::SharedPtr p_publisher_;
  rclcpp::Publisher<RGBMsg>::SharedPtr g_publisher_;
  rclcpp::Publisher<RGBMsg>::SharedPtr b_publisher_;
  rclcpp::Publisher<RGBMsg>::SharedPtr y_publisher_;
  rclcpp::Subscription<IlluminanceMsg>::SharedPtr illuminance_subscription_;
  rclcpp::Client<RobotInfoSrv>::SharedPtr robot_info_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  const std::size_t update_hz_;
  int ID_;
  std::string team_color_;
  uint16_t illuminance_;
  RobotInfo_t received_robot_info_;
  double camera_height_;
  bool is_robot_info_received_ = false;
  std::size_t no_recv_count_;

  PredEq_t blue_coef_;
  PredEq_t yellow_coef_;
  PredEq_t pink_coef_;
  PredEq_t green_coef_;

  void set_illuminance(IlluminanceMsg::SharedPtr illuminance_msg);
  void publish_color();
  double predict_output(std::string color, std::string composition);
  void request_robot_info(int& ID, std::string& team_color);
  void robot_info_callback(rclcpp::Client<RobotInfoSrv>::SharedFuture future);

  template <typename ClientType>
  bool is_service_available(const ClientType& client) {
    using namespace std::chrono_literals;

    // std::shared_ptrの型の場合、中身の型をチェック
    using ClientBaseType = typename std::remove_pointer<
        typename std::decay<decltype(*client)>::type>::type;

    static_assert(std::is_base_of<rclcpp::ClientBase, ClientBaseType>::value,
                  "ClientType must be a shared_ptr to a derived type of "
                  "rclcpp::ClientBase.");

    while (!client->wait_for_service(5s)) {
      if (!rclcpp::ok()) {
        return false;  // interrupted (service is unavailable)
      }
    }
    return true;  // service is available
  }

  void update();
};
}  // namespace active_marker
#endif
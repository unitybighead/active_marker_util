#ifndef LOGGER_HPP_
#define LOGGER_HPP_
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <type_traits>

#include "active_marker_msgs/msg/rgb.hpp"
#include "active_marker_msgs/srv/robot_info.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int16.hpp"

namespace active_marker {
class LoggerNode : public rclcpp::Node {
 public:
  struct RGB {
    std::uint8_t r;
    std::uint8_t g;
    std::uint8_t b;
  };

  struct PosRelation_t {
    double distance;
    double elev_ang;
  };

  struct RobotInfo_t {
    bool received;
    double x;
    double y;
    double height;
  };

  LoggerNode();

 private:
  using ColorMsg = active_marker_msgs::msg::RGB;
  using IlluminanceMsg = std_msgs::msg::UInt16;
  using Int16Msg = std_msgs::msg::Int16;
  using RobotInfoSrv = active_marker_msgs::srv::RobotInfo;

  const std::size_t update_hz_;

  rclcpp::Subscription<ColorMsg>::SharedPtr p_subscription_;
  rclcpp::Subscription<ColorMsg>::SharedPtr g_subscription_;
  rclcpp::Subscription<ColorMsg>::SharedPtr b_subscription_;
  rclcpp::Subscription<ColorMsg>::SharedPtr y_subscription_;
  rclcpp::Subscription<IlluminanceMsg>::SharedPtr illuminance_subscription_;
  rclcpp::Subscription<Int16Msg>::SharedPtr last_key_subscription_;
  rclcpp::Client<RobotInfoSrv>::SharedPtr robot_info_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  RGB blue_, yellow_, pink_, green_;
  uint16_t illuminance_;
  std::size_t no_recv_count_;
  int ID_;
  std::string team_color_;
  RobotInfo_t received_robot_info_;
  bool is_robot_info_received_ = false;

  void set_pink(ColorMsg::SharedPtr color_msg);
  void set_green(ColorMsg::SharedPtr color_msg);
  void set_blue(ColorMsg::SharedPtr color_msg);
  void set_yellow(ColorMsg::SharedPtr color_msg);
  void set_illuminance(IlluminanceMsg::SharedPtr illuminance_msg);
  void check_key(Int16Msg::SharedPtr msg);
  void write_config_csv();
  void request_robot_info(int& ID, std::string& team_color);
  void robot_info_callback(rclcpp::Client<RobotInfoSrv>::SharedFuture future);
  PosRelation_t calc_pos_relation(Eigen::Vector3d& pos1, Eigen::Vector3d& pos2);

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
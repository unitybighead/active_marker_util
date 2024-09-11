#include "camera_gui.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<active_marker::CameraGUINode>());
  rclcpp::shutdown();
  return 0;
}
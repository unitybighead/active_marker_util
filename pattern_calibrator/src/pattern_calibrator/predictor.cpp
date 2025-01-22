#include "predictor.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<active_marker::PredictorNode>());
  rclcpp::shutdown();
  return 0;
}
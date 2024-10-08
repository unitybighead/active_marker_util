#include "logger.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <fstream>
#include <ostream>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

namespace active_marker {

LoggerNode::LoggerNode() : Node("logger") {
  const auto qos = rclcpp::QoS(1).reliable();
  illuminance_subscription_ = this->create_subscription<IlluminanceMsg>(
      "illuminance", qos,
      std::bind(&LoggerNode::set_illuminance, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(1000ms / update_hz_,
                                   std::bind(&LoggerNode::update_frame, this));

  // Open the camera
  // if it's not connected DC1394, use default camera
  if (!dc1394_.is_available()) {
    RCLCPP_ERROR(this->get_logger(), "DC1394 initialize error");
    cap_.open(0);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open camera");
      rclcpp::shutdown();
    }
  }

  std::string package_path =
      ament_index_cpp::get_package_share_directory("pattern_logger");
  std::string config_path = package_path + "/config_path.yaml";
  try {
    YAML::Node path_node = YAML::LoadFile(config_path);
    color_csv_ = path_node["color"].as<std::string>();
    RCLCPP_INFO(this->get_logger(), "%s", color_csv_.c_str());
  } catch (const std::exception& e) {
    std::cerr << "Error:" << e.what() << std::endl;
  }

  // create a window and set callback functions
  cv::namedWindow("Pattern Logger", cv::WINDOW_NORMAL);
  cv::setMouseCallback("Pattern Logger", LoggerNode::onMouse, this);
}

void LoggerNode::onMouse(int event, int x, int y, int flags, void* userdata) {
  LoggerNode* self = static_cast<LoggerNode*>(userdata);
  static bool dragging = false;
  static cv::Point drag_start, drag_end;
  if (!self) return;

  if (event == cv::EVENT_LBUTTONDOWN || event == cv::EVENT_RBUTTONDOWN) {
    if (flags == cv::EVENT_FLAG_CTRLKEY) {  // drag
      dragging = true;
      drag_start = cv::Point(x, y);
    }

    else {
      cv::Mat& frame = self->frame_to_display_;
      if (x >= 0 && x < frame.cols && y >= 0 && y < frame.rows) {
        cv::Vec3b bgr_pixel = frame.at<cv::Vec3b>(y, x);
        cv::Mat bgr(1, 1, CV_8UC3, bgr_pixel);
        cv::Mat yuv;
        cv::cvtColor(bgr, yuv, cv::COLOR_BGR2YUV);
        cv::Vec3b yuv_pixel = yuv.at<cv::Vec3b>(0, 0);

        RGB rgb_data;
        YUV yuv_data;

        rgb_data.r = bgr_pixel[2];
        rgb_data.g = bgr_pixel[1];
        rgb_data.b = bgr_pixel[0];
        yuv_data.y = yuv_pixel[0];
        yuv_data.u = yuv_pixel[1];
        yuv_data.v = yuv_pixel[2];
        std::stringstream rgb_text, yuv_text;
        rgb_text << "R: " << static_cast<int>(rgb_data.r)
                 << " G: " << static_cast<int>(rgb_data.g)
                 << " B: " << static_cast<int>(rgb_data.b);
        yuv_text << "Y: " << static_cast<int>(yuv_data.y)
                 << " U: " << static_cast<int>(yuv_data.u)
                 << " V: " << static_cast<int>(yuv_data.v);

        if (self->state_color_ == StateColor::NONE) {
          RCLCPP_ERROR(self->get_logger(),
                       "Please select the color you want to calibrate");
          return;
        }

        static const std::map<StateColor, std::string> color_map = {
            {StateColor::BLUE, "blue"},
            {StateColor::YELLOW, "yellow"},
            {StateColor::PINK, "pink"},
            {StateColor::GREEN, "green"},
            {StateColor::NONE, "none"}};
        std::string color = color_map.at(self->state_color_);

        self->rgb_text_ = rgb_text.str();
        self->yuv_text_ = yuv_text.str();

        self->write_parameter_csv(color, rgb_data, yuv_data, self->color_csv_);
      }
    }
  }

  else if (event == cv::EVENT_MOUSEMOVE && dragging) {
    drag_end = cv::Point(x, y);
    // 矩形を描画（ドラッグ中の視覚フィードバック用）
    cv::Mat frame_copy = self->frame_to_display_.clone();
    cv::rectangle(frame_copy, drag_start, drag_end, cv::Scalar(255, 255, 0), 2);
    cv::imshow("Pattern Logger", frame_copy);
    cv::waitKey(1);
  }

  else if (event == cv::EVENT_LBUTTONUP && dragging) {
    dragging = false;
    drag_end = cv::Point(x, y);
    // 選択された領域の矩形を取得
    cv::Rect selected_rect(drag_start, drag_end);

    // 領域がフレーム内に収まっているか確認
    selected_rect &= cv::Rect(0, 0, self->frame_to_display_.cols,
                              self->frame_to_display_.rows);
    if (selected_rect.area() > 0) {
      self->selected_region_ = self->frame_to_display_(selected_rect);
      cv::resize(self->selected_region_, self->selected_region_,
                 self->frame_to_display_.size(), 0, 0,
                 cv::INTER_LINEAR);  // フレーム全体にリサイズ

      if (selected_rect.area() > 0) {
        // 選択された領域を保存してズーム表示
        self->selected_region_ = self->frame_to_display_(selected_rect);
        self->is_zoomed_ = true;
      }
    }
  }
}

void LoggerNode::set_illuminance(IlluminanceMsg::SharedPtr msg) {
  illuminance_ = (std::uint16_t)msg->data;
  no_recv_count_ = 0;
}

void LoggerNode::write_parameter_csv(const std::string color, const RGB rgb,
                                 const YUV yuv, const std::string filename) {
  if (no_recv_count_ > 2 * update_hz_) {
    RCLCPP_ERROR(this->get_logger(), "No receive illuminance topic");
    return;
  }

  if (state_color_ == StateColor::NONE) {
    RCLCPP_ERROR(this->get_logger(), "State color is NONE, skipping CSV write");
    return;
  }

  // デバッグログで値を確認
  RCLCPP_INFO(this->get_logger(),
              "Writing CSV: color=%s, R=%d, G=%d, B=%d, Y=%d, U=%d, V=%d",
              color.c_str(), rgb.r, rgb.g, rgb.b, yuv.y, yuv.u, yuv.v);

  try {
    std::ofstream file(filename, std::ios::app);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s",
                   filename.c_str());
      return;
    }

    // CSV形式でデータを追記
    file << color << "," << static_cast<int>(illuminance_) << ","
         << static_cast<int>(rgb.r) << "," << static_cast<int>(rgb.g) << ","
         << static_cast<int>(rgb.b) << "," << static_cast<int>(yuv.y) << ","
         << static_cast<int>(yuv.u) << "," << static_cast<int>(yuv.v) << "\n";

    // ファイルを閉じる
    file.close();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during CSV write: %s",
                 e.what());
  }
}

void LoggerNode::update_frame() {
  no_recv_count_++;
  cv::Mat frame;
  static bool first_frame = true;
  if (dc1394_.is_available()) {
    frame = dc1394_.capture();
  } else {
    cap_ >> frame;
  }

  if (frame.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Captured empty frame");
    rclcpp::shutdown();
  }
  if (first_frame) {
    cv::resizeWindow("Pattern Logger", frame.cols, frame.rows);
    first_frame = false;
  }

  frame_to_display_ = frame.clone();

  if (is_zoomed_) {
    cv::resize(selected_region_, selected_region_, frame_to_display_.size(), 0,
               0, cv::INTER_LINEAR);
    frame_to_display_ = selected_region_.clone();  // ズームした部分を表示
  }

  static const std::map<StateColor, std::string> color_map = {
      {StateColor::BLUE, "BLUE"},
      {StateColor::YELLOW, "YELLOW"},
      {StateColor::PINK, "PINK"},
      {StateColor::GREEN, "GREEN"},
      {StateColor::NONE, "NONE"}};
  std::string color_text = color_map.at(state_color_);

  cv::putText(frame_to_display_, color_text, cv::Point(10, 30),
              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
  cv::putText(frame_to_display_, rgb_text_, cv::Point(10, 50),
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
  cv::putText(frame_to_display_, yuv_text_, cv::Point(10, 70),
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

  cv::imshow("Pattern Logger", frame_to_display_);
  cv::waitKey(1);  // 少し待って次のフレームをキャプチャ

  int key = cv::waitKey(30);
  switch (key) {
    case 27:  // ESC
      rclcpp::shutdown();
      break;
    case 'b':
    case 'B':
      state_color_ = StateColor::BLUE;
      RCLCPP_INFO(this->get_logger(), "blue");
      break;
    case 'y':
    case 'Y':
      state_color_ = StateColor::YELLOW;
      RCLCPP_INFO(this->get_logger(), "yellow");
      break;
    case 'p':
    case 'P':
      state_color_ = StateColor::PINK;
      RCLCPP_INFO(this->get_logger(), "pink");
      break;
    case 'g':
    case 'G':
      state_color_ = StateColor::GREEN;
      RCLCPP_INFO(this->get_logger(), "green");
      break;
    case 'z':
    case 'Z':
      is_zoomed_ = false;
      break;
    case -1:
      break;
    default:
      state_color_ = StateColor::NONE;
      RCLCPP_INFO(this->get_logger(), "none");
      break;
  }
}

}  // namespace active_marker

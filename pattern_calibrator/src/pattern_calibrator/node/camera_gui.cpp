#include "camera_gui.hpp"

#include <sstream>

namespace active_marker {

CameraGUINode::CameraGUINode() : Node("camera_gui", "/am16") {
  const auto qos = rclcpp::QoS(1).best_effort();
  cur_color_publisher_ = this->create_publisher<ColorInfoMsg>("cur_color", qos);
  ref_color_publisher_ = this->create_publisher<ColorInfoMsg>("ref_color", qos);

  // タイマーによってフレームをキャプチャ・表示
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(20),
                              std::bind(&CameraGUINode::update_frame, this));

  // カメラを開く
  cap_.open(0);
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Unable to open camera");
    rclcpp::shutdown();
  }

  // ウィンドウを作成し、マウスコールバックを設定
  cv::namedWindow("Pattern Calibrator");
  cv::setMouseCallback("Pattern Calibrator", CameraGUINode::onMouse, this);
}

void CameraGUINode::onMouse(int event, int x, int y, int, void* userdata) {
  CameraGUINode* self = static_cast<CameraGUINode*>(userdata);
  if (!self) return;

  if (event == cv::EVENT_LBUTTONDOWN || event == cv::EVENT_RBUTTONDOWN) {
    cv::Mat& frame = self->frame_to_display_;
    if (x >= 0 && x < frame.cols && y >= 0 && y < frame.rows) {
      cv::Vec3b bgr_pixel = frame.at<cv::Vec3b>(y, x);
      cv::Mat bgr(1, 1, CV_8UC3, bgr_pixel);
      cv::Mat yuv;
      cv::cvtColor(bgr, yuv, cv::COLOR_BGR2YUV);
      cv::Vec3b yuv_pixel = yuv.at<cv::Vec3b>(0, 0);

      // YUVとRGBの値を文字列に変換
      std::ostringstream rgb_stream, yuv_stream;
      rgb_stream << "R: " << static_cast<int>(bgr_pixel[2])
                 << ", G: " << static_cast<int>(bgr_pixel[1])
                 << ", B: " << static_cast<int>(bgr_pixel[0]);
      yuv_stream << "Y: " << static_cast<int>(yuv_pixel[0])
                 << ", U: " << static_cast<int>(yuv_pixel[1])
                 << ", V: " << static_cast<int>(yuv_pixel[2]);

      self->rgb_text_ = rgb_stream.str();
      self->yuv_text_ = yuv_stream.str();

      auto color_msg = ColorInfoMsg();
      color_msg.rgb.r = bgr_pixel[2];
      color_msg.rgb.g = bgr_pixel[1];
      color_msg.rgb.b = bgr_pixel[0];
      color_msg.yuv.y = yuv_pixel[0];
      color_msg.yuv.u = yuv_pixel[1];
      color_msg.yuv.v = yuv_pixel[2];

      if (self->state_color_ == StateColor::NONE) {
        RCLCPP_ERROR(self->get_logger(),
                     "Please select the color you want to calibrate");
        return;
      }

      static const std::unordered_map<StateColor, std::string> color_map = {
          {StateColor::BLUE, "blue"},
          {StateColor::YELLOW, "yellow"},
          {StateColor::PINK, "pink"},
          {StateColor::GREEN, "green"}};

      color_msg.color = color_map.at(self->state_color_);

      if (event == cv::EVENT_LBUTTONDOWN) {
        self->cur_color_publisher_->publish(color_msg);
      } else if (event == cv::EVENT_RBUTTONDOWN) {
        self->ref_color_publisher_->publish(color_msg);
      }
    }
  }
}

void CameraGUINode::update_frame() {
  cv::Mat frame;
  cap_ >> frame;
  if (frame.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Captured empty frame");
    rclcpp::shutdown();
  }

  cv::flip(frame, frame, 1);
  frame_to_display_ = frame.clone();  // 表示用フレームをコピー

  cv::putText(frame_to_display_, rgb_text_, cv::Point(10, 30),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
  cv::putText(frame_to_display_, yuv_text_, cv::Point(10, 60),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
  cv::imshow("Pattern Calibrator", frame_to_display_);
  cv::waitKey(1);  // 少し待って次のフレームをキャプチャ

  int key = cv::waitKey(30);
  switch (key) {
    case 27:  // ESC
      rclcpp::shutdown();
      break;
    case 'b':
    case 'B':
      state_color_ = StateColor::BLUE;
      break;
    case 'y':
    case 'Y':
      state_color_ = StateColor::YELLOW;
      break;
    case 'p':
    case 'P':
      state_color_ = StateColor::PINK;
      break;
    case 'g':
    case 'G':
      state_color_ = StateColor::GREEN;
      break;
    case -1:
      break;
    default:
      state_color_ = StateColor::NONE;
      break;
  }
}

}  // namespace active_marker

#include "camera_gui.hpp"

namespace active_marker {

CameraGUINode::CameraGUINode() : Node("camera_gui") {
  // タイマーによってフレームをキャプチャ・表示
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(33),
                              std::bind(&CameraGUINode::update_frame, this));

  // カメラを開く
  cap_.open(0);
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Unable to open camera");
    rclcpp::shutdown();
  }

  // ウィンドウを作成し、マウスコールバックを設定
  cv::namedWindow("Camera Color Setting");
  cv::setMouseCallback("Camera Color Setting", CameraGUINode::onMouse, this);
}

void CameraGUINode::onMouse(int event, int x, int y, int, void* userdata) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    CameraGUINode* self = static_cast<CameraGUINode*>(userdata);
    if (self) {
      cv::Mat& frame = self->frame_to_display_;
      if (x >= 0 && x < frame.cols && y >= 0 && y < frame.rows) {
        cv::Vec3b bgr_pixel = frame.at<cv::Vec3b>(y, x);

        // BGRからYUVに変換
        cv::Mat bgr(1, 1, CV_8UC3, bgr_pixel);
        cv::Mat yuv;
        cv::cvtColor(bgr, yuv, cv::COLOR_BGR2YUV);
        cv::Vec3b yuv_pixel = yuv.at<cv::Vec3b>(0, 0);

        // YUVとRGBの値を文字列に変換
        self->yuv_text_ = "Y: " + std::to_string(yuv_pixel[0]) +
                          ", U: " + std::to_string(yuv_pixel[1]) +
                          ", V: " + std::to_string(yuv_pixel[2]);

        self->rgb_text_ = "R: " + std::to_string(bgr_pixel[2]) +
                          ", G: " + std::to_string(bgr_pixel[1]) +
                          ", B: " + std::to_string(bgr_pixel[0]);
      }
    }
  }

  else if (event == cv::EVENT_RBUTTONDOWN) {
  }
}

void CameraGUINode::update_frame() {
  // フレームをキャプチャ
  cv::Mat frame, buf;
  cap_ >> buf;
  cv::flip(buf, frame, 1);
  if (frame.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Captured empty frame");
    rclcpp::shutdown();
  }

  // グローバル変数を更新
  frame_to_display_ = frame.clone();  // 表示用フレームをコピー
  cv::putText(frame_to_display_, yuv_text_, cv::Point(10, 30),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
  cv::putText(frame_to_display_, rgb_text_, cv::Point(10, 60),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

  // フレームを表示
  cv::imshow("Camera Color Setting", frame_to_display_);
  cv::waitKey(1);  // 少し待って次のフレームをキャプチャ

  // ESCキーで終了
  if (cv::waitKey(15) == 27) {
    rclcpp::shutdown();
  }
}

}  // namespace active_marker

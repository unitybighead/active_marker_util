#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

// グローバル変数
cv::Mat frame_to_display;
std::string yuv_text = "";
std::string rgb_text = "";

// マウスクリック時のコールバック関数
void onMouse(int event, int x, int y, int, void* userdata) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    cv::Mat* frame = static_cast<cv::Mat*>(userdata);
    cv::Vec3b bgr_pixel = (*frame).at<cv::Vec3b>(y, x);

    // BGRからYUVに変換
    cv::Mat bgr(1, 1, CV_8UC3, bgr_pixel);
    cv::Mat yuv;
    cv::cvtColor(bgr, yuv, cv::COLOR_BGR2YUV);
    cv::Vec3b yuv_pixel = yuv.at<cv::Vec3b>(0, 0);

    // YUVとRGBの値を文字列に変換
    yuv_text = "Y: " + std::to_string(yuv_pixel[0]) +
               ", U: " + std::to_string(yuv_pixel[1]) +
               ", V: " + std::to_string(yuv_pixel[2]);

    rgb_text = "R: " + std::to_string(bgr_pixel[2]) +
               ", G: " + std::to_string(bgr_pixel[1]) +
               ", B: " + std::to_string(bgr_pixel[0]);
  }
}

int main() {
  // USBカメラを開く (カメラIDが0のカメラを開く)
  cv::VideoCapture cap(0);
  if (!cap.isOpened()) {
    std::cerr << "Error: Unable to open camera!" << std::endl;
    return -1;
  }

  // ウィンドウを作成
  cv::namedWindow("USB Camera");

  // マウスコールバック関数の設定
  cv::setMouseCallback("USB Camera", onMouse, &frame_to_display);

  while (true) {
    // フレームをキャプチャ
    cv::Mat frame;
    cap >> frame;
    if (frame.empty()) {
      std::cerr << "Error: Empty frame captured!" << std::endl;
      break;
    }

    // 表示用フレームをコピー
    frame_to_display = frame.clone();

    // テキストをオーバーレイ
    if (!yuv_text.empty()) {
      cv::putText(frame_to_display, yuv_text, cv::Point(10, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    }
    if (!rgb_text.empty()) {
      cv::putText(frame_to_display, rgb_text, cv::Point(10, 60),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    }

    // フレームを表示
    cv::imshow("USB Camera", frame_to_display);

    // ESCキーで終了
    if (cv::waitKey(30) == 27) {
      break;
    }
  }

  // カメラをリリースしてウィンドウを閉じる
  cap.release();
  cv::destroyAllWindows();

  return 0;
}

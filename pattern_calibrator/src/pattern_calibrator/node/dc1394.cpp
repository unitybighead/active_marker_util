#include "dc1394.hpp"

#include <iostream>

DC1394::DC1394() {
  d_ = dc1394_new();
  dc1394camera_list_t* list;
  dc1394_camera_enumerate(d_, &list);

  if (list->num == 0) {
    std::cerr << "No DC1394 found" << std::endl;
    return;
  }

  camera_ = dc1394_camera_new(d_, list->ids[0].guid);
  dc1394_camera_free_list(list);

  // Error handling for camera initialization
  if (camera_ == nullptr) {
    std::cerr << "Failed to initialize camera" << std::endl;
    dc1394_free(d_);
    return;
  }

  // カメラの設定を取得
  dc1394video_modes_t video_modes;
  dc1394_video_get_supported_modes(camera_, &video_modes);

  dc1394_video_set_mode(camera_, video_modes.modes[0]);
  dc1394_video_set_iso_speed(camera_, DC1394_ISO_SPEED_100);
  dc1394_video_set_framerate(camera_, DC1394_FRAMERATE_30);

  dc1394_capture_setup(camera_, 4, DC1394_CAPTURE_FLAGS_DEFAULT);
  dc1394_video_set_transmission(camera_, DC1394_ON);
  is_available_ = true;
}

bool DC1394::is_available() { return is_available_; }

cv::Mat DC1394::capture() {
  cv::Mat img;
  dc1394video_frame_t* frame;
  dc1394_capture_dequeue(camera_, DC1394_CAPTURE_POLICY_WAIT, &frame);

  if (frame == nullptr) {
    std::cerr << "Failed to dequeue frame" << std::endl;
    img.release();
    return img;
  }

  print_frame_info(frame);

  switch (frame->color_coding) {
    case DC1394_COLOR_CODING_RGB8:
      img = cv::Mat(frame->size[1], frame->size[0], CV_8UC3, frame->image);
      cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
      break;
    case DC1394_COLOR_CODING_YUV422:
      img = cv::Mat(frame->size[1], frame->size[0], CV_8UC2, frame->image);
      cv::cvtColor(img, img, cv::COLOR_YUV2BGR_Y422);
      break;
    case DC1394_COLOR_CODING_YUV444:
      img = cv::Mat(frame->size[1], frame->size[0], CV_8UC3, frame->image);
      cv::cvtColor(img, img, cv::COLOR_YUV2BGR);
      break;
    case DC1394_COLOR_CODING_MONO8:
      img = cv::Mat(frame->size[1], frame->size[0], CV_8UC1, frame->image);
      cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
      break;
    case DC1394_COLOR_CODING_RAW8:
      img = cv::Mat(frame->size[1], frame->size[0], CV_8UC1, frame->image);
      cv::cvtColor(img, img, cv::COLOR_BayerBG2BGR);
      break;
    default:
      std::cerr << "Unsupported color coding: " << frame->color_coding
                << std::endl;
      img = cv::Mat(frame->size[1], frame->size[0], CV_8UC1, frame->image);
      break;
  }

  dc1394_capture_enqueue(camera_, frame);
  return img;
}

DC1394::~DC1394() {
  is_available_ = false;
  dc1394_video_set_transmission(camera_, DC1394_OFF);
  dc1394_capture_stop(camera_);
  dc1394_camera_free(camera_);
  dc1394_free(d_);
}

void DC1394::print_frame_info(dc1394video_frame_t* frame) {
  std::cout << "Frame info:" << std::endl;
  std::cout << "  Size: " << frame->size[0] << "x" << frame->size[1]
            << std::endl;
  std::cout << "  Color coding: " << frame->color_coding << std::endl;
  std::cout << "  Data depth: " << frame->data_depth << " bits" << std::endl;
  std::cout << "  Stride: " << frame->stride << " bytes" << std::endl;
  std::cout << "  Total bytes: " << frame->total_bytes << std::endl;
}

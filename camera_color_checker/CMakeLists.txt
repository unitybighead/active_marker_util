cmake_minimum_required(VERSION 3.10)

# プロジェクト名と使用するC++のバージョン
project(USB_Camera_Display_YUV_RGB)
set(CMAKE_CXX_STANDARD 11)

# OpenCVの設定
find_package(OpenCV REQUIRED)

# 実行ファイルの追加
add_executable(camera_display main.cpp)

# OpenCVライブラリをリンク
target_link_libraries(camera_display ${OpenCV_LIBS})

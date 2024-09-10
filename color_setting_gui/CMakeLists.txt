cmake_minimum_required(VERSION 3.14)

project(color_setting_manager)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(${PROJECT_NAME}
    main.cpp
    color_settingJJPNua.h
    color_setting_gui.cpp
    color_setting_gui.hpp
)

target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

cmake_minimum_required(VERSION 3.8)
project(all_publisher)

set(CMAKE_CXX_STANDARD 20)
# Find ament_cmake_auto first
find_package(ament_cmake_auto REQUIRED)

# Use ament_cmake_auto to set up the package
ament_auto_find_build_dependencies()

# Add main library
file(GLOB MAIN_LIB_SOURCES "src/*.cpp")
ament_auto_add_library(${PROJECT_NAME} SHARED ${MAIN_LIB_SOURCES})

# Find executables automatically
file(GLOB EXECUTABLE_SOURCES "*.cpp")

foreach(EXECUTABLE_SOURCE ${EXECUTABLE_SOURCES})
  get_filename_component(EXECUTABLE_NAME ${EXECUTABLE_SOURCE} NAME_WE)
  
  ament_auto_add_executable(${EXECUTABLE_NAME} ${EXECUTABLE_SOURCE})
endforeach()
target_link_libraries(${PROJECT_NAME} yaml-cpp)

install(FILES
  config_path.yaml
  DESTINATION share/${PROJECT_NAME}
)

# Install executables and libraries
ament_auto_package()
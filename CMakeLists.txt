cmake_minimum_required(VERSION 3.8)
project(projectb_program)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH "/path/to/ros2_robotiqgripper/install" ${CMAKE_PREFIX_PATH})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_robotiqgripper REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# test1.cpp 実行ファイルの設定
add_executable(test1 src/test1.cpp)
target_include_directories(test1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(test1 PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  test1
  "moveit_ros_planning_interface"
  "rclcpp"
  "ros2_robotiqgripper"
  "OpenCV"
  "cv_bridge"
)

install(TARGETS test1
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


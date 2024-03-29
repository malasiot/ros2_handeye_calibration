cmake_minimum_required(VERSION 3.8)
project(handeye_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(handeye_calibration_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED core imgproc aruco highgui calib3d)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

add_executable(handeye_action_server
    src/handeye_robot.cpp
    src/handeye_robot.hpp
    src/robot_commander.cpp
    src/robot_commander.hpp
    src/util.cpp
    )

target_include_directories(handeye_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(handeye_action_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(handeye_action_server
  rclcpp  rclcpp_action
  rclcpp_components std_srvs std_msgs sensor_msgs geometry_msgs tf2
  tf2_ros tf2_eigen image_transport
  moveit_ros_planning_interface
  cv_bridge Eigen3
  handeye_calibration_msgs
)
target_link_libraries(handeye_action_server ${OpenCV_LIBS})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

add_executable(handeye_gui
    src/handeye_gui.cpp
    src/handeye_gui.hpp
    src/image_widget.cpp
    src/image_widget.hpp
    )
target_include_directories(handeye_gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(handeye_gui PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(handeye_gui
  rclcpp std_msgs sensor_msgs tf2
  tf2_ros Eigen3
  rclcpp_action
  rclcpp_components
  cv_bridge
  handeye_calibration_msgs
)
target_link_libraries(handeye_gui Qt::Core Qt::Gui Qt::Widgets ${OpenCV_LIBS})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

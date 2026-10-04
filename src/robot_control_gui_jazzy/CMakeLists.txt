cmake_minimum_required(VERSION 3.16)
project(robot_control_gui_jazzy
  VERSION 0.1.0
  DESCRIPTION "ROS2 Jazzy + Qt5 robot control GUI (mentorpi)"
  LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----- ROS2 deps -----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# ----- Qt5 -----
find_package(Qt5 REQUIRED COMPONENTS
  Core
  Gui
  Widgets
  Network
)

# Threads (for MultiThreadedExecutor)
find_package(Threads REQUIRED)

# ----- Sources -----
set(UI_SOURCES
  src/ui/main_window.cpp
  src/ui/control_panel.cpp
  src/ui/navigation_panel.cpp
  src/ui/mapping_panel.cpp
  src/ui/settings_panel.cpp
  src/ui/robot_status_panel.cpp
  src/ui/joystick_widget.cpp
  src/ui/speed_dashboard.cpp
  src/ui/teleop_panel.cpp
  src/ui/robot_view.cpp
  src/ui/goal_setting_dialog.cpp
  src/ui/initial_pose_dialog.cpp
  src/ui/planner_settings_dialog.cpp
  src/ui/mjpeg_stream.cpp
)

set(UI_HEADERS
  include/robot_control_gui_jazzy/ui/main_window.h
  include/robot_control_gui_jazzy/ui/control_panel.h
  include/robot_control_gui_jazzy/ui/navigation_panel.h
  include/robot_control_gui_jazzy/ui/mapping_panel.h
  include/robot_control_gui_jazzy/ui/settings_panel.h
  include/robot_control_gui_jazzy/ui/robot_status_panel.h
  include/robot_control_gui_jazzy/ui/joystick_widget.h
  include/robot_control_gui_jazzy/ui/speed_dashboard.h
  include/robot_control_gui_jazzy/ui/teleop_panel.h
  include/robot_control_gui_jazzy/ui/robot_view.h
  include/robot_control_gui_jazzy/ui/goal_setting_dialog.h
  include/robot_control_gui_jazzy/ui/initial_pose_dialog.h
  include/robot_control_gui_jazzy/ui/planner_settings_dialog.h
  include/robot_control_gui_jazzy/ui/mjpeg_stream.h
)

set(ROS_SOURCES
  src/ros/robot_controller.cpp
  src/ros/diagnostics_watcher.cpp
  src/ros/executor_registration.cpp
)

set(ROS_HEADERS
  include/robot_control_gui_jazzy/ros/robot_controller.h
  include/robot_control_gui_jazzy/ros/diagnostics_watcher.h
  include/robot_control_gui_jazzy/ros/topic_names.h
)

# ----- Library (so we can also expose a component if needed) -----
add_library(${PROJECT_NAME}_lib SHARED
  ${ROS_SOURCES}
  ${ROS_HEADERS}
  ${UI_SOURCES}
  ${UI_HEADERS}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  diagnostic_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# NOTE: ament_target_dependencies already calls plain target_link_libraries
# internally, so we must stay plain too — no PUBLIC keyword.
target_link_libraries(${PROJECT_NAME}_lib
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::Network
    Threads::Threads
)

# ----- Executable (Qt + rclcpp in main) -----
add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_action
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Stay plain — see comment on _lib above.
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::Network
)

# ----- Install -----
install(
  TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # ---- Pure-C++ gtest for the static helpers (no rclcpp runtime needed) ----
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(Qt5 REQUIRED COMPONENTS Core Gui)

  ament_add_gtest(test_robot_controller_helpers
    test/test_helpers.cpp
  )
  target_include_directories(test_robot_controller_helpers PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${nav_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
  )
  ament_target_dependencies(test_robot_controller_helpers
    nav_msgs
    sensor_msgs
    geometry_msgs
  )
  target_link_libraries(test_robot_controller_helpers
    Qt5::Core
    Qt5::Gui
  )
endif()

ament_package()

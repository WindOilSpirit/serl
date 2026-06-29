cmake_minimum_required(VERSION 3.8)
project(serl_franka_ros2_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT console_bridge_DIR AND EXISTS "/usr/lib/x86_64-linux-gnu/console_bridge/cmake")
  set(console_bridge_DIR "/usr/lib/x86_64-linux-gnu/console_bridge/cmake" CACHE PATH
      "Path to console_bridge CMake config")
endif()
if(NOT TINYXML2_LIBRARY AND EXISTS "/usr/lib/x86_64-linux-gnu/libtinyxml2.so")
  set(TINYXML2_LIBRARY "/usr/lib/x86_64-linux-gnu/libtinyxml2.so" CACHE FILEPATH
      "Path to TinyXML2 library")
endif()
if(NOT TINYXML2_INCLUDE_DIR AND EXISTS "/usr/include/tinyxml2.h")
  set(TINYXML2_INCLUDE_DIR "/usr/include" CACHE PATH "Path to TinyXML2 include directory")
endif()
if(NOT spdlog_DIR AND EXISTS "/usr/lib/x86_64-linux-gnu/cmake/spdlog")
  set(spdlog_DIR "/usr/lib/x86_64-linux-gnu/cmake/spdlog" CACHE PATH
      "Path to spdlog CMake config")
endif()
if(NOT urdfdom_DIR AND EXISTS "/opt/ros/humble/lib/x86_64-linux-gnu/urdfdom/cmake")
  set(urdfdom_DIR "/opt/ros/humble/lib/x86_64-linux-gnu/urdfdom/cmake" CACHE PATH
      "Path to urdfdom CMake config")
elseif(NOT urdfdom_DIR AND EXISTS "/usr/lib/x86_64-linux-gnu/urdfdom/cmake")
  set(urdfdom_DIR "/usr/lib/x86_64-linux-gnu/urdfdom/cmake" CACHE PATH
      "Path to urdfdom CMake config")
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/joint_sine_torque_test_controller.cpp
  src/serl_cartesian_impedance_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  franka_semantic_components
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

pluginlib_export_plugin_description_file(
  controller_interface serl_franka_ros2_control_plugins.xml
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES serl_franka_ros2_control_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  controller_interface
  franka_semantic_components
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

ament_package()

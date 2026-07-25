cmake_minimum_required(VERSION 3.16)
project(dm_imu LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

#
# 普通 ROS 2 IMU 节点
#
add_executable(${PROJECT_NAME}_node
  src/bsp_crc.cpp
  src/uart_port.cpp
  src/imu_driver.cpp
  src/test_imu.cpp
)

target_include_directories(${PROJECT_NAME}_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
    rclcpp::rclcpp
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
)

#
# ros2_control SensorInterface 插件
#
add_library(dm_imu_hardware SHARED
  src/hardware/imu_hardware.cpp
  src/uart_port.cpp
  src/bsp_crc.cpp
)

target_include_directories(dm_imu_hardware
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(dm_imu_hardware
  PRIVATE
    rclcpp::rclcpp
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(
  hardware_interface
  resource/dm_imu_hardware_plugin.xml
)

#
# 安装
#
install(
  TARGETS
    ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    dm_imu_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

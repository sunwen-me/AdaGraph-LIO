cmake_minimum_required(VERSION 3.16)
project(basic VERSION 2.0 LANGUAGES CXX C)

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE STRING "Suppress developer warnings" FORCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Glog REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
find_package(Threads REQUIRED)


file(GLOB_RECURSE BASIC_SOURCES "src/*.cpp")
file(GLOB_RECURSE BASIC_HEADERS "include/*.h" "include/*.hpp")

add_library(${PROJECT_NAME} SHARED
  ${BASIC_SOURCES}
  ${BASIC_HEADERS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${rclcpp_LIBRARIES}
  gflags
  glog
  Threads::Threads
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${rclcpp_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3 PCL)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_manifold test/test_manifold.cpp)
  target_link_libraries(test_manifold ${PROJECT_NAME})

  ament_add_gtest(test_ring_buffer test/test_ring_buffer.cpp)
  target_link_libraries(test_ring_buffer ${PROJECT_NAME})
endif()

ament_package()

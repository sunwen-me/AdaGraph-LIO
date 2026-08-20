cmake_minimum_required(VERSION 3.16)
project(hybrid_slam VERSION 2.0 LANGUAGES CXX C)

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE STRING "Suppress developer warnings" FORCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
find_package(Glog REQUIRED)
find_package(basic REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

# HybridSLAM dependencies
find_package(GTSAM REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(TBB QUIET)
find_package(OpenMP REQUIRED)

option(HYBRID_SLAM_ENABLE_KISS_MATCHER
  "Build the optional KISS-Matcher relocalization dependency" ON)

# Small Point-LIO is vendored at a fixed upstream commit and is used only by
# the Point-LIO production executable.  It stays in-process so HybridSLAM
# receives the exact accepted point-to-plane observations without ROS message
# synchronization or loss of Hessian information.
set(HYBRID_SLAM_VENDOR_POINT_LIO_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/small_point_lio")
set(HYBRID_SLAM_POINT_LIO_GENERATED_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/point_lio_generated")
file(MAKE_DIRECTORY "${HYBRID_SLAM_POINT_LIO_GENERATED_DIR}")
configure_file(
  "${HYBRID_SLAM_VENDOR_POINT_LIO_DIR}/include/param_deliver.h.in"
  "${HYBRID_SLAM_POINT_LIO_GENERATED_DIR}/param_deliver.h"
  @ONLY
)

set(HYBRID_SLAM_VENDOR_FAST_GICP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/fast_gicp")
set(HYBRID_SLAM_USE_VENDOR_FAST_GICP OFF)
if(EXISTS "${HYBRID_SLAM_VENDOR_FAST_GICP_DIR}/include/fast_gicp/gicp/fast_vgicp.hpp")
  set(HYBRID_SLAM_USE_VENDOR_FAST_GICP ON)
else()
  find_package(fast_gicp QUIET)
endif()

set(HYBRID_SLAM_VENDOR_KISS_MATCHER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/kiss_matcher")
set(HYBRID_SLAM_USE_VENDOR_KISS_MATCHER OFF)
if(HYBRID_SLAM_ENABLE_KISS_MATCHER AND
   EXISTS "${HYBRID_SLAM_VENDOR_KISS_MATCHER_DIR}/core/kiss_matcher/KISSMatcher.hpp")
  set(HYBRID_SLAM_USE_VENDOR_KISS_MATCHER ON)
endif()

set(HYBRID_SLAM_VENDOR_ROBIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/robin")
set(HYBRID_SLAM_USE_VENDOR_ROBIN OFF)
if(HYBRID_SLAM_ENABLE_KISS_MATCHER AND
   EXISTS "${HYBRID_SLAM_VENDOR_ROBIN_DIR}/include/robin/robin.hpp")
  set(HYBRID_SLAM_USE_VENDOR_ROBIN ON)
endif()

if(HYBRID_SLAM_ENABLE_KISS_MATCHER AND NOT HYBRID_SLAM_USE_VENDOR_KISS_MATCHER)
  find_package(kiss_matcher QUIET)
endif()

if(TARGET kiss_matcher::kiss_matcher_core)
  message(STATUS "kiss_matcher found, enabling KISS-Matcher relocalization")
elseif(HYBRID_SLAM_USE_VENDOR_KISS_MATCHER)
  message(STATUS "vendored kiss_matcher found, enabling KISS-Matcher relocalization")
else()
  if(HYBRID_SLAM_ENABLE_KISS_MATCHER)
    message(STATUS "kiss_matcher not found, KISS-Matcher relocalization disabled")
  else()
    message(STATUS "KISS-Matcher relocalization explicitly disabled")
  endif()
endif()
if(HYBRID_SLAM_USE_VENDOR_FAST_GICP)
  message(STATUS "vendored fast_gicp found, enabling FastVGICP prior-map localization")
elseif(fast_gicp_FOUND)
  message(STATUS "fast_gicp found, enabling FastVGICP prior-map localization")
else()
  message(STATUS "fast_gicp not found, prior-map localization will use PCL GICP fallback")
endif()


set(HYBRID_SLAM_POINT_LIO_ALGORITHM_SRC
  src/lio/bevplace2_loop_detector.cpp
  src/lio/hybrid_slam_feature_extractor.cpp
  src/lio/kiss_matcher_relocalizer.cpp
  src/lio/loop_closure_plugin.cpp
  src/lio/pcl_relocalizer_adapters.cpp
  src/lio/pcl_loop_closure.cpp
  src/lio/pcl_submap_relocalizer.cpp
  src/lio/pcl_submap_relocalization_scheduler.cpp
  src/lio/prior_map_localization.cpp
  src/lio/runtime_memory.cpp
  src/lio/scan_context_loop_detector.cpp
)
set(HYBRID_SLAM_ROS_PUBLICATION_SRC
  src/ros/ground_segmentation_filter.cpp
  src/ros/occupancy_grid_publisher.cpp
  src/ros/publication_bridge.cpp
)
set(HYBRID_SLAM_POINT_LIO_PIPELINE_SRC
  src/system/frame_health_gate.cpp
  src/system/global_alignment_publisher.cpp
  src/system/map_save_facade.cpp
  src/system/point_lio_pipeline.cpp
)

add_library(hybrid_slam_mapping_core STATIC
  src/mapping/map_manager.cpp
  src/mapping/map_snapshot.cpp
  src/mapping/submap_geometry.cpp
  src/mapping/submap_index.cpp
  src/mapping/submap_recovery_seed.cpp
  src/mapping/occupancy_grid.cpp
  src/mapping/snapshot_products.cpp
)
target_compile_features(hybrid_slam_mapping_core PUBLIC cxx_std_20)
target_include_directories(hybrid_slam_mapping_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hybrid_slam_mapping_core PUBLIC Eigen3::Eigen)

add_library(hybrid_slam_map_io STATIC
  src/io/map_io.cpp
)
target_compile_features(hybrid_slam_map_io PUBLIC cxx_std_20)
target_include_directories(hybrid_slam_map_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hybrid_slam_map_io PUBLIC
  hybrid_slam_mapping_core
  Eigen3::Eigen
  Threads::Threads
)

add_library(hybrid_slam_localization_core STATIC
  src/localization/composed_relocalizer.cpp
  src/localization/localization_map.cpp
  src/localization/relocalization_update.cpp
  src/localization/relocalization_scheduler.cpp
)
target_compile_features(hybrid_slam_localization_core PUBLIC cxx_std_20)
target_include_directories(hybrid_slam_localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hybrid_slam_localization_core PUBLIC
  hybrid_slam_mapping_core
  Eigen3::Eigen
  Threads::Threads
)

add_library(hybrid_slam_system_core STATIC
  src/system/global_alignment_state.cpp
  src/system/dataset_runner.cpp
  src/system/slam_system.cpp
)
target_compile_features(hybrid_slam_system_core PUBLIC cxx_std_20)
target_include_directories(hybrid_slam_system_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hybrid_slam_system_core PUBLIC
  hybrid_slam_localization_core
  hybrid_slam_mapping_core
  Eigen3::Eigen
)

add_library(hybrid_slam_backend_core STATIC
  src/backend/backend_scheduler.cpp
  src/backend/loop_closure_scheduler.cpp
  src/backend/odometry_covariance_estimator.cpp
)
target_compile_features(hybrid_slam_backend_core PUBLIC cxx_std_20)
target_include_directories(hybrid_slam_backend_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hybrid_slam_backend_core PUBLIC
  hybrid_slam_mapping_core
  Eigen3::Eigen
  Threads::Threads
)

add_library(hybrid_slam_isam2_optimizer STATIC
  src/backend/isam2_optimizer.cpp
)
target_compile_features(hybrid_slam_isam2_optimizer PUBLIC cxx_std_20)
target_include_directories(hybrid_slam_isam2_optimizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hybrid_slam_isam2_optimizer PUBLIC
  hybrid_slam_backend_core
  Eigen3::Eigen
  gtsam
)

add_library(hybrid_slam_logging STATIC
  src/logging/logging.cpp
)
target_compile_features(hybrid_slam_logging PUBLIC cxx_std_20)
target_include_directories(hybrid_slam_logging PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hybrid_slam_logging PUBLIC
  glog
  gflags
)

# ROS-only conversion, TF and lifecycle command adapters are isolated from
# the ROS-independent core targets.
add_library(hybrid_slam_ros2_adapter STATIC
  src/ros/parameter_bridge.cpp
  src/ros/livox_point_time.cpp
  src/ros/sensor_bridge.cpp
  src/ros/tf_bridge.cpp
  src/ros/service_bridge.cpp
)
target_compile_features(hybrid_slam_ros2_adapter PUBLIC cxx_std_20)
target_include_directories(hybrid_slam_ros2_adapter PUBLIC
  include
  3rdparty
  3rdparty/ground_segmentation/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${livox_ros_driver2_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)
target_link_libraries(hybrid_slam_ros2_adapter PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${livox_ros_driver2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  basic::basic_core
  hybrid_slam_system_core
)
add_library(point_lio_vendor STATIC
  3rdparty/small_point_lio/src/small_point_lio/estimator.cpp
  3rdparty/small_point_lio/src/small_point_lio/parameters.cpp
  3rdparty/small_point_lio/src/small_point_lio/preprocess.cpp
  3rdparty/small_point_lio/src/small_point_lio/small_point_lio.cpp
  3rdparty/small_point_lio/src/util/voxelgrid_sampling.cpp
)
target_compile_features(point_lio_vendor PUBLIC cxx_std_20)
target_compile_definitions(point_lio_vendor PUBLIC HAVE_LIVOX_DRIVER)
target_include_directories(point_lio_vendor PUBLIC
  ${HYBRID_SLAM_VENDOR_POINT_LIO_DIR}/include
  ${HYBRID_SLAM_VENDOR_POINT_LIO_DIR}/3rdparty
  ${HYBRID_SLAM_VENDOR_POINT_LIO_DIR}/src
  ${HYBRID_SLAM_POINT_LIO_GENERATED_DIR}
)
target_link_libraries(point_lio_vendor PUBLIC
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

if(HYBRID_SLAM_USE_VENDOR_FAST_GICP)
  add_library(fast_gicp_vendor SHARED
    3rdparty/fast_gicp/src/fast_gicp/gicp/lsq_registration.cpp
    3rdparty/fast_gicp/src/fast_gicp/gicp/fast_gicp.cpp
    3rdparty/fast_gicp/src/fast_gicp/gicp/fast_gicp_st.cpp
    3rdparty/fast_gicp/src/fast_gicp/gicp/fast_vgicp.cpp
  )

  target_include_directories(fast_gicp_vendor PUBLIC
    ${HYBRID_SLAM_VENDOR_FAST_GICP_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )

  target_link_libraries(fast_gicp_vendor PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
  )

  if(OpenMP_CXX_FOUND)
    target_link_libraries(fast_gicp_vendor PUBLIC OpenMP::OpenMP_CXX)
  endif()
endif()

if(HYBRID_SLAM_USE_VENDOR_ROBIN)
  set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(BUILD_MATLAB_BINDINGS OFF CACHE BOOL "" FORCE)
  set(USE_SYSTEM_EIGEN3 ON CACHE BOOL "" FORCE)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/3rdparty/robin/include")
  add_subdirectory(3rdparty/robin EXCLUDE_FROM_ALL)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(HYBRID_SLAM_USE_VENDOR_KISS_MATCHER)
  set(USE_SYSTEM_EIGEN3 ON CACHE BOOL "" FORCE)
  set(USE_SYSTEM_TBB ON CACHE BOOL "" FORCE)
  set(USE_SYSTEM_ROBIN ON CACHE BOOL "" FORCE)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)
  add_subdirectory(3rdparty/kiss_matcher EXCLUDE_FROM_ALL)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_library(hybrid_slam_point_lio_algorithms STATIC
  ${HYBRID_SLAM_POINT_LIO_ALGORITHM_SRC}
)
target_compile_features(hybrid_slam_point_lio_algorithms PUBLIC cxx_std_20)
target_include_directories(hybrid_slam_point_lio_algorithms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  3rdparty
  3rdparty/ground_segmentation/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

target_link_libraries(hybrid_slam_point_lio_algorithms PUBLIC
  hybrid_slam_backend_core
  hybrid_slam_localization_core
  hybrid_slam_isam2_optimizer
  hybrid_slam_logging
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  Threads::Threads
  gtsam
  basic::basic_core
)

if(TARGET kiss_matcher::kiss_matcher_core)
  target_compile_definitions(hybrid_slam_point_lio_algorithms PUBLIC HAS_KISS_MATCHER)
  target_link_libraries(hybrid_slam_point_lio_algorithms PUBLIC kiss_matcher::kiss_matcher_core)
endif()

if(HYBRID_SLAM_USE_VENDOR_FAST_GICP)
  target_compile_definitions(hybrid_slam_point_lio_algorithms PUBLIC HAS_FAST_GICP)
  target_link_libraries(hybrid_slam_point_lio_algorithms PUBLIC fast_gicp_vendor)
elseif(fast_gicp_FOUND)
  target_compile_definitions(hybrid_slam_point_lio_algorithms PUBLIC HAS_FAST_GICP)
  if(TARGET fast_gicp::fast_gicp)
    target_link_libraries(hybrid_slam_point_lio_algorithms PUBLIC fast_gicp::fast_gicp)
  elseif(TARGET fast_gicp)
    target_link_libraries(hybrid_slam_point_lio_algorithms PUBLIC fast_gicp)
  else()
    target_include_directories(hybrid_slam_point_lio_algorithms PUBLIC ${fast_gicp_INCLUDE_DIRS})
    target_link_libraries(hybrid_slam_point_lio_algorithms PUBLIC ${fast_gicp_LIBRARIES})
  endif()
endif()

# Standard mapping/localization applications use the explicit publication
# adapter.
add_library(hybrid_slam_ros_publication STATIC
  ${HYBRID_SLAM_ROS_PUBLICATION_SRC}
)
target_compile_features(hybrid_slam_ros_publication PUBLIC cxx_std_20)
target_include_directories(hybrid_slam_ros_publication PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  3rdparty
  3rdparty/ground_segmentation/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${livox_ros_driver2_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)
target_link_libraries(hybrid_slam_ros_publication PUBLIC
  hybrid_slam_ros2_adapter
  hybrid_slam_backend_core
  hybrid_slam_map_io
  hybrid_slam_system_core
  hybrid_slam_mapping_core
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  Threads::Threads
  gtsam
  ${std_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${pcl_ros_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${livox_ros_driver2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

# Point-LIO's downstream mapping/localization composition is ROS-independent;
# applications compose it with the explicit ROS adapters at their boundary.
add_library(hybrid_slam_point_lio_pipeline STATIC
  ${HYBRID_SLAM_POINT_LIO_PIPELINE_SRC}
)
target_compile_features(hybrid_slam_point_lio_pipeline PUBLIC cxx_std_20)
target_include_directories(hybrid_slam_point_lio_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  3rdparty
  3rdparty/ground_segmentation/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(hybrid_slam_point_lio_pipeline PUBLIC
  hybrid_slam_point_lio_algorithms
  hybrid_slam_backend_core
  hybrid_slam_isam2_optimizer
  hybrid_slam_system_core
  hybrid_slam_map_io
  hybrid_slam_mapping_core
  hybrid_slam_logging
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  Threads::Threads
  gtsam
  basic::basic_core
)
if(TARGET kiss_matcher::kiss_matcher_core)
  target_compile_definitions(hybrid_slam_point_lio_pipeline PUBLIC HAS_KISS_MATCHER)
  target_link_libraries(hybrid_slam_point_lio_pipeline PUBLIC kiss_matcher::kiss_matcher_core)
endif()
if(HYBRID_SLAM_USE_VENDOR_FAST_GICP)
  target_compile_definitions(hybrid_slam_point_lio_pipeline PUBLIC HAS_FAST_GICP)
  target_link_libraries(hybrid_slam_point_lio_pipeline PUBLIC fast_gicp_vendor)
elseif(fast_gicp_FOUND)
  target_compile_definitions(hybrid_slam_point_lio_pipeline PUBLIC HAS_FAST_GICP)
  if(TARGET fast_gicp::fast_gicp)
    target_link_libraries(hybrid_slam_point_lio_pipeline PUBLIC fast_gicp::fast_gicp)
  elseif(TARGET fast_gicp)
    target_link_libraries(hybrid_slam_point_lio_pipeline PUBLIC fast_gicp)
  else()
    target_include_directories(hybrid_slam_point_lio_pipeline PUBLIC ${fast_gicp_INCLUDE_DIRS})
    target_link_libraries(hybrid_slam_point_lio_pipeline PUBLIC ${fast_gicp_LIBRARIES})
  endif()
endif()

add_library(point_lio_adapter STATIC
  src/lio/point_lio_frontend.cpp
)
target_compile_features(point_lio_adapter PUBLIC cxx_std_20)
target_include_directories(point_lio_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(point_lio_adapter PUBLIC
  point_lio_vendor
  basic::basic_core
  Eigen3::Eigen
  Threads::Threads
)

add_library(hybrid_slam_point_lio_recovery STATIC
  src/lio/point_lio_recovery_coordinator.cpp
)
target_compile_features(hybrid_slam_point_lio_recovery PUBLIC cxx_std_20)
target_include_directories(hybrid_slam_point_lio_recovery PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hybrid_slam_point_lio_recovery PUBLIC
  point_lio_adapter
  hybrid_slam_point_lio_pipeline
  hybrid_slam_mapping_core
  Eigen3::Eigen
  Threads::Threads
)

add_executable(point_lio_node src/apps/point_lio_node.cpp)
target_link_libraries(point_lio_node
  point_lio_adapter
  hybrid_slam_point_lio_recovery
  hybrid_slam_logging
  hybrid_slam_ros2_adapter
  hybrid_slam_ros_publication
  hybrid_slam_system_core
  hybrid_slam_point_lio_pipeline
)

# ROS-free production composition driven by the same typed text packets as
# DatasetRunner. YAML is parsed only at this application boundary; all
# frontend/system/downstream targets remain ROS-independent.
add_executable(hybrid_slam_dataset_runner src/apps/dataset_runner_node.cpp)
target_compile_features(hybrid_slam_dataset_runner PUBLIC cxx_std_20)
target_include_directories(hybrid_slam_dataset_runner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
  ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ground_segmentation/include
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(hybrid_slam_dataset_runner PRIVATE
  hybrid_slam_system_core
  point_lio_adapter
  hybrid_slam_point_lio_pipeline
  hybrid_slam_logging
  yaml-cpp::yaml-cpp
  Eigen3::Eigen
  Threads::Threads
  gtsam
  basic::basic_core
)

install(TARGETS
  hybrid_slam_mapping_core
  hybrid_slam_map_io
  hybrid_slam_localization_core
  hybrid_slam_system_core
  hybrid_slam_backend_core
  hybrid_slam_isam2_optimizer
  hybrid_slam_logging
  hybrid_slam_ros2_adapter
  hybrid_slam_ros_publication
  hybrid_slam_point_lio_algorithms
  hybrid_slam_point_lio_pipeline
  hybrid_slam_point_lio_recovery
  point_lio_adapter
  point_lio_vendor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(TARGET fast_gicp_vendor)
  install(TARGETS
    fast_gicp_vendor
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

if(TARGET pmc)
  install(TARGETS
    pmc
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

install(TARGETS
  point_lio_node
  hybrid_slam_dataset_runner
  DESTINATION lib/${PROJECT_NAME}
)

# Nothing in the install space consumes these headers (the build uses the
# source tree directly); skip test/binding/doc directories to keep the
# Jetson install artifact small.
install(DIRECTORY
  include/hybrid_slam
  DESTINATION include
)

install(DIRECTORY
  include/lio
  include/ros
  include/common
  3rdparty/
  DESTINATION include
  PATTERN "tests" EXCLUDE
  PATTERN "matlab" EXCLUDE
  PATTERN "python" EXCLUDE
  PATTERN "docs" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME test_summarize_tum_diagnostics
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/test_summarize_tum.py
  )
  add_test(
    NAME test_slam_regression_acceptance
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/test_run_slam_regression.py
  )
  add_test(
    NAME test_core_boundary_dependencies
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/verify_core_boundaries.py
            ${CMAKE_CURRENT_SOURCE_DIR}/../..
  )
  add_test(
    NAME test_launch_mode_validation
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch_mode_validation.py
  )

  ament_add_gtest(test_livox_point_time test/test_livox_point_time.cpp)
  target_link_libraries(test_livox_point_time
    hybrid_slam_ros2_adapter
  )

  ament_add_gtest(test_sensor_bridge test/test_sensor_bridge.cpp)
  target_link_libraries(test_sensor_bridge
    hybrid_slam_ros2_adapter
  )

  ament_add_gtest(test_service_bridge test/test_service_bridge.cpp)
  target_link_libraries(test_service_bridge
    hybrid_slam_ros2_adapter
    hybrid_slam_system_core
  )

  ament_add_gtest(test_slam_system test/test_slam_system.cpp)
  target_link_libraries(test_slam_system
    hybrid_slam_system_core
    Eigen3::Eigen
  )

  ament_add_gtest(test_frontend_lifecycle test/test_frontend_lifecycle.cpp)
  target_link_libraries(test_frontend_lifecycle
    point_lio_adapter
    Eigen3::Eigen
  )

  ament_add_gtest(test_dataset_runner test/test_dataset_runner.cpp)
  target_link_libraries(test_dataset_runner
    hybrid_slam_system_core
    point_lio_adapter
    Eigen3::Eigen
  )

  ament_add_gtest(test_frontend_result test/test_frontend_result.cpp)
  target_include_directories(test_frontend_result PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_frontend_result Eigen3::Eigen)

  ament_add_gtest(test_frontend_health test/test_frontend_health.cpp)
  target_include_directories(test_frontend_health PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_frontend_health Eigen3::Eigen)

  ament_add_gtest(
    test_point_lio_recovery_transition
    test/test_point_lio_recovery_transition.cpp
  )

  ament_add_gtest(
    test_map_save_facade
    test/test_map_save_facade.cpp
  )
  target_link_libraries(test_map_save_facade
    hybrid_slam_point_lio_pipeline
    Eigen3::Eigen
  )

  ament_add_gtest(
    test_frame_health_gate
    test/test_frame_health_gate.cpp
  )
  target_link_libraries(test_frame_health_gate
    hybrid_slam_point_lio_pipeline
    Eigen3::Eigen
  )

  ament_add_gtest(
    test_pcl_submap_relocalizer
    test/test_pcl_submap_relocalizer.cpp
  )
  target_link_libraries(test_pcl_submap_relocalizer
    hybrid_slam_point_lio_algorithms
    Eigen3::Eigen
  )
  target_link_libraries(test_point_lio_recovery_transition
    hybrid_slam_point_lio_pipeline
    hybrid_slam_point_lio_recovery
    Eigen3::Eigen
  )

  ament_add_gtest(
    test_odometry_rate_limiter
    test/test_odometry_rate_limiter.cpp
  )
  target_include_directories(test_odometry_rate_limiter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_core_domain test/test_core_domain.cpp)
  target_link_libraries(test_core_domain
    hybrid_slam_backend_core
    hybrid_slam_map_io
    hybrid_slam_localization_core
    hybrid_slam_system_core
    Eigen3::Eigen
  )

  if(TARGET kiss_matcher::kiss_matcher_core)
    ament_add_gtest(
      test_kiss_matcher_sampling
      test/test_kiss_matcher_sampling.cpp
    )
    target_link_libraries(test_kiss_matcher_sampling
      kiss_matcher::kiss_matcher_core
      Eigen3::Eigen
    )
  endif()

  ament_add_gtest(test_isam2_optimizer test/test_isam2_optimizer.cpp)
  target_link_libraries(test_isam2_optimizer
    hybrid_slam_isam2_optimizer
    Eigen3::Eigen
  )

  ament_add_gtest(
    test_loop_closure_scheduler
    test/test_loop_closure_scheduler.cpp
  )
  target_link_libraries(test_loop_closure_scheduler
    hybrid_slam_backend_core
    Eigen3::Eigen
  )

  ament_add_gtest(
    test_scan_context_loop_detector
    test/test_scan_context_loop_detector.cpp
  )
  target_link_libraries(test_scan_context_loop_detector
    hybrid_slam_point_lio_algorithms
    hybrid_slam_mapping_core
    Eigen3::Eigen
  )

  ament_add_gtest(
    test_ground_segmentation_filter
    test/test_ground_segmentation_filter.cpp
  )
  target_link_libraries(
    test_ground_segmentation_filter
    hybrid_slam_ros_publication
  )

  ament_add_gtest(
    test_online_occupancy_timing
    test/test_online_occupancy_timing.cpp
  )
  target_link_libraries(
    test_online_occupancy_timing
    hybrid_slam_ros_publication
  )

  ament_add_gtest(
    test_global_map_publication
    test/test_global_map_publication.cpp
  )
  target_link_libraries(
    test_global_map_publication
    hybrid_slam_ros_publication
  )
endif()

ament_package()

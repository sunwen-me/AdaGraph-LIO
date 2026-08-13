cmake_minimum_required(VERSION 3.16)
project(super_lio VERSION 2.0 LANGUAGES CXX C)

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE STRING "Suppress developer warnings" FORCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

# AdaGraph-LIO dependencies
find_package(GTSAM REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(TBB QUIET)
find_package(OpenMP REQUIRED)

option(SUPER_LIO_ENABLE_KISS_MATCHER
  "Build the optional KISS-Matcher relocalization dependency" ON)

# Small Point-LIO is vendored at a fixed upstream commit and is used only by
# the alternate point_lio_node executable.  It stays in-process so AdaGraph
# receives the exact accepted point-to-plane observations without ROS message
# synchronization or loss of Hessian information.
set(SUPER_LIO_VENDOR_POINT_LIO_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/small_point_lio")
set(SUPER_LIO_POINT_LIO_GENERATED_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/point_lio_generated")
file(MAKE_DIRECTORY "${SUPER_LIO_POINT_LIO_GENERATED_DIR}")
configure_file(
  "${SUPER_LIO_VENDOR_POINT_LIO_DIR}/include/param_deliver.h.in"
  "${SUPER_LIO_POINT_LIO_GENERATED_DIR}/param_deliver.h"
  @ONLY
)

set(SUPER_LIO_VENDOR_FAST_GICP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/fast_gicp")
set(SUPER_LIO_USE_VENDOR_FAST_GICP OFF)
if(EXISTS "${SUPER_LIO_VENDOR_FAST_GICP_DIR}/include/fast_gicp/gicp/fast_vgicp.hpp")
  set(SUPER_LIO_USE_VENDOR_FAST_GICP ON)
else()
  find_package(fast_gicp QUIET)
endif()

set(SUPER_LIO_VENDOR_KISS_MATCHER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/kiss_matcher")
set(SUPER_LIO_USE_VENDOR_KISS_MATCHER OFF)
if(SUPER_LIO_ENABLE_KISS_MATCHER AND
   EXISTS "${SUPER_LIO_VENDOR_KISS_MATCHER_DIR}/core/kiss_matcher/KISSMatcher.hpp")
  set(SUPER_LIO_USE_VENDOR_KISS_MATCHER ON)
endif()

set(SUPER_LIO_VENDOR_ROBIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/robin")
set(SUPER_LIO_USE_VENDOR_ROBIN OFF)
if(SUPER_LIO_ENABLE_KISS_MATCHER AND
   EXISTS "${SUPER_LIO_VENDOR_ROBIN_DIR}/include/robin/robin.hpp")
  set(SUPER_LIO_USE_VENDOR_ROBIN ON)
endif()

if(SUPER_LIO_ENABLE_KISS_MATCHER AND NOT SUPER_LIO_USE_VENDOR_KISS_MATCHER)
  find_package(kiss_matcher QUIET)
endif()

if(TARGET kiss_matcher::kiss_matcher_core)
  message(STATUS "kiss_matcher found, enabling KISS-Matcher relocalization")
elseif(SUPER_LIO_USE_VENDOR_KISS_MATCHER)
  message(STATUS "vendored kiss_matcher found, enabling KISS-Matcher relocalization")
else()
  if(SUPER_LIO_ENABLE_KISS_MATCHER)
    message(STATUS "kiss_matcher not found, KISS-Matcher relocalization disabled")
  else()
    message(STATUS "KISS-Matcher relocalization explicitly disabled")
  endif()
endif()
if(SUPER_LIO_USE_VENDOR_FAST_GICP)
  message(STATUS "vendored fast_gicp found, enabling FastVGICP prior-map localization")
elseif(fast_gicp_FOUND)
  message(STATUS "fast_gicp found, enabling FastVGICP prior-map localization")
else()
  message(STATUS "fast_gicp not found, prior-map localization will use PCL GICP fallback")
endif()


rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CloudPose.msg
  msg/CloudPose2.msg
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

set(ADAGRAPH_LEGACY_BACKEND_SRC
  src/lio/adagraph_backend.cpp
)
set(ADAGRAPH_LEGACY_ALGORITHM_SRC
  src/lio/adagraph_feature_extractor.cpp
  src/lio/kiss_matcher_relocalizer.cpp
  src/lio/pcl_relocalizer_adapters.cpp
  src/lio/prior_map_localization.cpp
)
set(ADAGRAPH_LEGACY_COMPAT_SRC
  src/lio/legacy_algorithm_config_adapter.cpp
)

set(ADAGRAPH_ROS_COMPOSITION_SRC
  src/ros/ROSWrapper.cpp
)
set(ADAGRAPH_ROS_INPUT_SRC
  src/ros/legacy_input_bridge.cpp
)
set(ADAGRAPH_ROS_PUBLICATION_SRC
  src/ros/ground_segmentation_filter.cpp
  src/ros/occupancy_grid_publisher.cpp
  src/ros/publication_bridge.cpp
)
set(ADAGRAPH_LEGACY_DOWNSTREAM_SRC
  src/system/legacy_downstream_pipeline.cpp
)
# The old ``lio`` name is retained as an interface target below for source and
# launch compatibility.  The implementation is split into a ROS-free legacy
# algorithm target and a ROS/downstream composition target so that algorithm
# tests and future non-ROS applications do not inherit ROSWrapper, publishers
# or lifecycle adapters merely by linking the legacy backend.

add_library(adagraph_legacy_config STATIC
  src/lio/params.cpp
)
target_compile_features(adagraph_legacy_config PUBLIC cxx_std_20)
target_compile_definitions(adagraph_legacy_config PRIVATE
  "ROOT=\"${CMAKE_CURRENT_SOURCE_DIR}/\""
)
target_include_directories(adagraph_legacy_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  3rdparty
  ${EIGEN3_INCLUDE_DIRS}
  ${basic_INCLUDE_DIRS}
)
target_link_libraries(adagraph_legacy_config PUBLIC
  Eigen3::Eigen
  ${basic_LIBRARIES}
)

add_library(adagraph_mapping_core STATIC
  src/mapping/map_manager.cpp
  src/mapping/map_snapshot.cpp
  src/mapping/occupancy_grid.cpp
  src/mapping/snapshot_products.cpp
)
target_compile_features(adagraph_mapping_core PUBLIC cxx_std_20)
target_include_directories(adagraph_mapping_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(adagraph_mapping_core PUBLIC Eigen3::Eigen)

add_library(adagraph_map_io STATIC
  src/io/map_io.cpp
)
target_compile_features(adagraph_map_io PUBLIC cxx_std_20)
target_include_directories(adagraph_map_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(adagraph_map_io PUBLIC
  adagraph_mapping_core
  Eigen3::Eigen
  Threads::Threads
)

add_library(adagraph_localization_core STATIC
  src/localization/composed_relocalizer.cpp
  src/localization/localization_map.cpp
  src/localization/relocalization_update.cpp
  src/localization/relocalization_scheduler.cpp
)
target_compile_features(adagraph_localization_core PUBLIC cxx_std_20)
target_include_directories(adagraph_localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(adagraph_localization_core PUBLIC
  adagraph_mapping_core
  Eigen3::Eigen
  Threads::Threads
)

add_library(adagraph_system_core STATIC
  src/system/global_alignment_state.cpp
  src/system/dataset_runner.cpp
  src/system/slam_system.cpp
)
target_compile_features(adagraph_system_core PUBLIC cxx_std_20)
target_include_directories(adagraph_system_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(adagraph_system_core PUBLIC
  adagraph_localization_core
  adagraph_mapping_core
  Eigen3::Eigen
)

add_library(adagraph_backend_core STATIC
  src/backend/backend_scheduler.cpp
  src/backend/odometry_covariance_estimator.cpp
)
target_compile_features(adagraph_backend_core PUBLIC cxx_std_20)
target_include_directories(adagraph_backend_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(adagraph_backend_core PUBLIC
  adagraph_mapping_core
  Eigen3::Eigen
  Threads::Threads
)

add_library(adagraph_isam2_optimizer STATIC
  src/backend/isam2_optimizer.cpp
)
target_compile_features(adagraph_isam2_optimizer PUBLIC cxx_std_20)
target_include_directories(adagraph_isam2_optimizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(adagraph_isam2_optimizer PUBLIC
  adagraph_backend_core
  Eigen3::Eigen
  gtsam
)

# ROS-only conversion, TF and lifecycle command adapters are isolated from
# the ROS-independent core targets. The legacy lio composition links this
# adapter explicitly during the strangler migration.
add_library(adagraph_ros2_adapter STATIC
  src/ros/legacy_parameter_bridge.cpp
  ${ADAGRAPH_ROS_INPUT_SRC}
  src/ros/livox_point_time.cpp
  src/ros/sensor_bridge.cpp
  src/ros/tf_bridge.cpp
  src/ros/service_bridge.cpp
)
target_compile_features(adagraph_ros2_adapter PUBLIC cxx_std_20)
target_compile_definitions(adagraph_ros2_adapter PRIVATE
  # Only the compatibility parameter loader needs the source-tree fallback.
  # Keep this legacy deployment behavior local instead of exporting an
  # absolute source path to every core/frontend target.
  "ROOT=\"${CMAKE_CURRENT_SOURCE_DIR}/\""
)
target_include_directories(adagraph_ros2_adapter PUBLIC
  include
  3rdparty
  3rdparty/ground_segmentation/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
  ${basic_INCLUDE_DIRS}
  ${livox_ros_driver2_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)
target_link_libraries(adagraph_ros2_adapter PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  glog
  ${rclcpp_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${livox_ros_driver2_LIBRARIES}
  ${basic_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  adagraph_system_core
)

# The old global-parameter loader remains available for external rollback
# callers, but it is compiled as a separate ROS compatibility target.  The
# standard adapter translation unit is built without params.h/g_* symbols.
add_library(adagraph_legacy_global_ros_compat STATIC
  src/ros/legacy_global_parameter_bridge.cpp
)
target_compile_features(adagraph_legacy_global_ros_compat PUBLIC cxx_std_20)
target_include_directories(adagraph_legacy_global_ros_compat PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  3rdparty
  ${EIGEN3_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
  ${basic_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(adagraph_legacy_global_ros_compat PUBLIC
  adagraph_legacy_config
  Eigen3::Eigen
  glog
  ${rclcpp_LIBRARIES}
  ${basic_LIBRARIES}
)

# Transitional Super-LIO frontend target. Its implementation consumes only
# typed input or injected legacy providers; ROSWrapper stays in the adapter
# and compatibility composition targets below.
add_library(adagraph_super_lio_frontend STATIC
  src/lio/ESKF.cpp
  src/lio/super_lio.cpp
)
target_compile_features(adagraph_super_lio_frontend PUBLIC cxx_std_20)
target_include_directories(adagraph_super_lio_frontend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  3rdparty
  3rdparty/ground_segmentation/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
  ${basic_INCLUDE_DIRS}
)
target_link_libraries(adagraph_super_lio_frontend PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  glog
  Threads::Threads
  ${basic_LIBRARIES}
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
  ${SUPER_LIO_VENDOR_POINT_LIO_DIR}/include
  ${SUPER_LIO_VENDOR_POINT_LIO_DIR}/3rdparty
  ${SUPER_LIO_VENDOR_POINT_LIO_DIR}/src
  ${SUPER_LIO_POINT_LIO_GENERATED_DIR}
)
target_link_libraries(point_lio_vendor PUBLIC
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

if(SUPER_LIO_USE_VENDOR_FAST_GICP)
  add_library(fast_gicp_vendor SHARED
    3rdparty/fast_gicp/src/fast_gicp/gicp/lsq_registration.cpp
    3rdparty/fast_gicp/src/fast_gicp/gicp/fast_gicp.cpp
    3rdparty/fast_gicp/src/fast_gicp/gicp/fast_gicp_st.cpp
    3rdparty/fast_gicp/src/fast_gicp/gicp/fast_vgicp.cpp
  )

  target_include_directories(fast_gicp_vendor PUBLIC
    ${SUPER_LIO_VENDOR_FAST_GICP_DIR}/include
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

if(SUPER_LIO_USE_VENDOR_ROBIN)
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

if(SUPER_LIO_USE_VENDOR_KISS_MATCHER)
  set(USE_SYSTEM_EIGEN3 ON CACHE BOOL "" FORCE)
  set(USE_SYSTEM_TBB ON CACHE BOOL "" FORCE)
  set(USE_SYSTEM_ROBIN ON CACHE BOOL "" FORCE)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)
  add_subdirectory(3rdparty/kiss_matcher EXCLUDE_FROM_ALL)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_library(adagraph_legacy_algorithms STATIC
  ${ADAGRAPH_LEGACY_ALGORITHM_SRC}
)
target_compile_features(adagraph_legacy_algorithms PUBLIC cxx_std_20)
target_include_directories(adagraph_legacy_algorithms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  3rdparty
  3rdparty/ground_segmentation/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${basic_INCLUDE_DIRS}
)

target_link_libraries(adagraph_legacy_algorithms PUBLIC
  adagraph_backend_core
  adagraph_localization_core
  adagraph_isam2_optimizer
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  glog
  Threads::Threads
  gtsam
  ${basic_LIBRARIES}
)

if(TARGET kiss_matcher::kiss_matcher_core)
  target_compile_definitions(adagraph_legacy_algorithms PUBLIC HAS_KISS_MATCHER)
  target_link_libraries(adagraph_legacy_algorithms PUBLIC kiss_matcher::kiss_matcher_core)
endif()

# Isolate the old AdaGraph facade itself from the reusable legacy algorithm
# adapters. Standard downstream no longer links this target; relocation and
# legacy backend tests retain it as an explicit rollback surface.
add_library(adagraph_legacy_backend STATIC
  ${ADAGRAPH_LEGACY_BACKEND_SRC}
)
target_compile_features(adagraph_legacy_backend PUBLIC cxx_std_20)
target_include_directories(adagraph_legacy_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  3rdparty
  3rdparty/ground_segmentation/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${basic_INCLUDE_DIRS}
)
target_link_libraries(adagraph_legacy_backend PUBLIC
  adagraph_legacy_algorithms
  adagraph_backend_core
  adagraph_isam2_optimizer
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  glog
  Threads::Threads
  gtsam
  ${basic_LIBRARIES}
)

# Compatibility-only global-parameter entry points.  Keeping this tiny target
# separate means the algorithm archive and ROS-free downstream composition can
# be instantiated from explicit configs without linking params.cpp.
add_library(adagraph_legacy_compat STATIC
  ${ADAGRAPH_LEGACY_COMPAT_SRC}
)
target_compile_features(adagraph_legacy_compat PUBLIC cxx_std_20)
target_include_directories(adagraph_legacy_compat PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  3rdparty
  ${EIGEN3_INCLUDE_DIRS}
  ${basic_INCLUDE_DIRS}
)
target_link_libraries(adagraph_legacy_compat PUBLIC
  adagraph_legacy_algorithms
  adagraph_legacy_backend
  adagraph_legacy_config
  Eigen3::Eigen
  ${basic_LIBRARIES}
)

if(SUPER_LIO_USE_VENDOR_FAST_GICP)
  target_compile_definitions(adagraph_legacy_algorithms PUBLIC HAS_FAST_GICP)
  target_link_libraries(adagraph_legacy_algorithms PUBLIC fast_gicp_vendor)
elseif(fast_gicp_FOUND)
  target_compile_definitions(adagraph_legacy_algorithms PUBLIC HAS_FAST_GICP)
  if(TARGET fast_gicp::fast_gicp)
    target_link_libraries(adagraph_legacy_algorithms PUBLIC fast_gicp::fast_gicp)
  elseif(TARGET fast_gicp)
    target_link_libraries(adagraph_legacy_algorithms PUBLIC fast_gicp)
  else()
    target_include_directories(adagraph_legacy_algorithms PUBLIC ${fast_gicp_INCLUDE_DIRS})
    target_link_libraries(adagraph_legacy_algorithms PUBLIC ${fast_gicp_LIBRARIES})
  endif()
endif()

# Standard mapping/localization applications use the explicit publication
# adapter without inheriting the compatibility ROSWrapper implementation.
add_library(adagraph_ros_publication STATIC
  ${ADAGRAPH_ROS_PUBLICATION_SRC}
)
target_compile_features(adagraph_ros_publication PUBLIC cxx_std_20)
target_include_directories(adagraph_ros_publication PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  3rdparty
  3rdparty/ground_segmentation/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${basic_INCLUDE_DIRS}
  ${livox_ros_driver2_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)
target_link_libraries(adagraph_ros_publication PUBLIC
  adagraph_ros2_adapter
  adagraph_backend_core
  adagraph_map_io
  adagraph_system_core
  adagraph_mapping_core
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  glog
  Threads::Threads
  ${cpp_typesupport_target}
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
  ${basic_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

# Compatibility ROS composition. This target intentionally retains the
# ROSWrapper implementation for external compatibility callers.
add_library(adagraph_ros_composition STATIC
  ${ADAGRAPH_ROS_COMPOSITION_SRC}
)
target_compile_features(adagraph_ros_composition PUBLIC cxx_std_20)
target_include_directories(adagraph_ros_composition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  3rdparty
  3rdparty/ground_segmentation/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${basic_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)
target_link_libraries(adagraph_ros_composition PUBLIC
  adagraph_ros2_adapter
  adagraph_ros_publication
  adagraph_backend_core
  adagraph_system_core
  adagraph_mapping_core
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  glog
  Threads::Threads
  ${cpp_typesupport_target}
  ${std_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${pcl_ros_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${basic_LIBRARIES}
)

# Transitional downstream composition.  The implementation still adapts the
# legacy AdaGraph/PCL algorithms and process-wide compatibility parameters, but
# its output/session effects are explicit callbacks.  It therefore does not
# include or link ROS; applications compose it with adagraph_ros_composition
# when they need publishers and services.
add_library(adagraph_legacy_downstream STATIC
  ${ADAGRAPH_LEGACY_DOWNSTREAM_SRC}
)
target_compile_features(adagraph_legacy_downstream PUBLIC cxx_std_20)
target_include_directories(adagraph_legacy_downstream PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  3rdparty
  3rdparty/ground_segmentation/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${basic_INCLUDE_DIRS}
)
target_link_libraries(adagraph_legacy_downstream PUBLIC
  adagraph_legacy_algorithms
  adagraph_backend_core
  adagraph_isam2_optimizer
  adagraph_system_core
  adagraph_map_io
  adagraph_mapping_core
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  glog
  Threads::Threads
  gtsam
  ${basic_LIBRARIES}
)
if(TARGET kiss_matcher::kiss_matcher_core)
  target_compile_definitions(adagraph_legacy_downstream PUBLIC HAS_KISS_MATCHER)
  target_link_libraries(adagraph_legacy_downstream PUBLIC kiss_matcher::kiss_matcher_core)
endif()
if(SUPER_LIO_USE_VENDOR_FAST_GICP)
  target_compile_definitions(adagraph_legacy_downstream PUBLIC HAS_FAST_GICP)
  target_link_libraries(adagraph_legacy_downstream PUBLIC fast_gicp_vendor)
elseif(fast_gicp_FOUND)
  target_compile_definitions(adagraph_legacy_downstream PUBLIC HAS_FAST_GICP)
  if(TARGET fast_gicp::fast_gicp)
    target_link_libraries(adagraph_legacy_downstream PUBLIC fast_gicp::fast_gicp)
  elseif(TARGET fast_gicp)
    target_link_libraries(adagraph_legacy_downstream PUBLIC fast_gicp)
  else()
    target_include_directories(adagraph_legacy_downstream PUBLIC ${fast_gicp_INCLUDE_DIRS})
    target_link_libraries(adagraph_legacy_downstream PUBLIC ${fast_gicp_LIBRARIES})
  endif()
endif()

# Compatibility name.  New applications should link the coherent targets
# above; existing tests/launch integrations may continue to link ``lio`` while
# receiving the same split dependency graph transitively.
add_library(lio INTERFACE)
target_link_libraries(lio INTERFACE
  adagraph_legacy_algorithms
  adagraph_legacy_compat
  adagraph_legacy_global_ros_compat
  adagraph_legacy_downstream
  adagraph_ros_composition
)

add_library(point_lio_adapter STATIC
  src/lio/point_lio_frontend.cpp
)
target_compile_features(point_lio_adapter PUBLIC cxx_std_20)
target_include_directories(point_lio_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(point_lio_adapter PUBLIC
  point_lio_vendor
  basic::basic_core
  Eigen3::Eigen
  Threads::Threads
)

add_executable(super_lio_node src/apps/super_lio_node.cpp)
target_link_libraries(super_lio_node
  adagraph_ros2_adapter
  adagraph_ros_publication
  adagraph_system_core
  adagraph_legacy_downstream
  adagraph_super_lio_frontend
  ${cpp_typesupport_target}
)

add_executable(point_lio_node src/apps/point_lio_node.cpp)
target_link_libraries(point_lio_node
  point_lio_adapter
  adagraph_ros2_adapter
  adagraph_ros_publication
  adagraph_system_core
  adagraph_legacy_downstream
  ${cpp_typesupport_target}
)

# ROS-free production composition driven by the same typed text packets as
# DatasetRunner. YAML is parsed only at this application boundary; all
# frontend/system/downstream targets remain ROS-independent.
add_executable(adagraph_dataset_runner src/apps/dataset_runner_node.cpp)
target_compile_features(adagraph_dataset_runner PUBLIC cxx_std_20)
target_include_directories(adagraph_dataset_runner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
  ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ground_segmentation/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${basic_INCLUDE_DIRS}
)
target_link_libraries(adagraph_dataset_runner PRIVATE
  adagraph_system_core
  adagraph_super_lio_frontend
  point_lio_adapter
  adagraph_legacy_downstream
  yaml-cpp::yaml-cpp
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  glog
  Threads::Threads
  gtsam
  ${basic_LIBRARIES}
)

install(TARGETS
  adagraph_mapping_core
  adagraph_legacy_config
  adagraph_map_io
  adagraph_localization_core
  adagraph_system_core
  adagraph_backend_core
  adagraph_isam2_optimizer
  adagraph_legacy_compat
  adagraph_legacy_global_ros_compat
  adagraph_ros2_adapter
  adagraph_ros_publication
  adagraph_super_lio_frontend
  adagraph_legacy_algorithms
  adagraph_legacy_backend
  adagraph_legacy_downstream
  adagraph_ros_composition
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
  super_lio_node
  point_lio_node
  adagraph_dataset_runner
  DESTINATION lib/${PROJECT_NAME}
)

# Nothing in the install space consumes these headers (the build uses the
# source tree directly); skip test/binding/doc directories to keep the
# Jetson install artifact small.
install(DIRECTORY
  include/
  3rdparty/
  DESTINATION include/${PROJECT_NAME}
  PATTERN "tests" EXCLUDE
  PATTERN "matlab" EXCLUDE
  PATTERN "python" EXCLUDE
  PATTERN "docs" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
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
    NAME test_core_boundary_dependencies
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/verify_core_boundaries.py
            ${CMAKE_CURRENT_SOURCE_DIR}/../..
  )

  ament_add_gtest(test_adagraph_backend test/test_adagraph_backend.cpp TIMEOUT 120)
  target_link_libraries(test_adagraph_backend
    adagraph_legacy_backend
  )

  ament_add_gtest(test_octvoxmap test/test_octvoxmap.cpp TIMEOUT 120)
  target_include_directories(test_octvoxmap PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ground_segmentation/include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${basic_INCLUDE_DIRS}
  )
  target_link_libraries(test_octvoxmap Eigen3::Eigen ${basic_LIBRARIES})

  ament_add_gtest(test_livox_point_time test/test_livox_point_time.cpp)
  target_link_libraries(test_livox_point_time
    adagraph_ros2_adapter
    ${cpp_typesupport_target}
  )

  ament_add_gtest(test_sensor_bridge test/test_sensor_bridge.cpp)
  target_link_libraries(test_sensor_bridge
    adagraph_ros2_adapter
    ${cpp_typesupport_target}
  )

  ament_add_gtest(test_service_bridge test/test_service_bridge.cpp)
  target_link_libraries(test_service_bridge
    adagraph_ros2_adapter
    adagraph_system_core
    ${cpp_typesupport_target}
  )

  ament_add_gtest(test_slam_system test/test_slam_system.cpp)
  target_link_libraries(test_slam_system
    adagraph_system_core
    Eigen3::Eigen
  )

  ament_add_gtest(test_frontend_lifecycle test/test_frontend_lifecycle.cpp)
  target_link_libraries(test_frontend_lifecycle
    adagraph_super_lio_frontend
    point_lio_adapter
    Eigen3::Eigen
  )

  ament_add_gtest(test_dataset_runner test/test_dataset_runner.cpp)
  target_link_libraries(test_dataset_runner
    adagraph_system_core
    adagraph_super_lio_frontend
    point_lio_adapter
    Eigen3::Eigen
  )

  ament_add_gtest(test_frontend_result test/test_frontend_result.cpp)
  target_include_directories(test_frontend_result PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_frontend_result Eigen3::Eigen)

  ament_add_gtest(test_core_domain test/test_core_domain.cpp)
  target_link_libraries(test_core_domain
    adagraph_backend_core
    adagraph_map_io
    adagraph_localization_core
    adagraph_system_core
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
    adagraph_isam2_optimizer
    Eigen3::Eigen
  )

  ament_add_gtest(
    test_ground_segmentation_filter
    test/test_ground_segmentation_filter.cpp
  )
  target_link_libraries(
    test_ground_segmentation_filter
    adagraph_ros_composition
    ${cpp_typesupport_target}
  )
endif()

ament_package()

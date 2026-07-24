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
add_definitions(-DROOT=\"${CMAKE_CURRENT_SOURCE_DIR}/\")


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

# AdaGraph-LIO dependencies
find_package(GTSAM REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(TBB QUIET)
find_package(OpenMP QUIET)

set(SUPER_LIO_VENDOR_FAST_GICP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/fast_gicp")
set(SUPER_LIO_USE_VENDOR_FAST_GICP OFF)
if(EXISTS "${SUPER_LIO_VENDOR_FAST_GICP_DIR}/include/fast_gicp/gicp/fast_vgicp.hpp")
  set(SUPER_LIO_USE_VENDOR_FAST_GICP ON)
else()
  find_package(fast_gicp QUIET)
endif()

set(SUPER_LIO_VENDOR_KISS_MATCHER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/kiss_matcher")
set(SUPER_LIO_USE_VENDOR_KISS_MATCHER OFF)
if(EXISTS "${SUPER_LIO_VENDOR_KISS_MATCHER_DIR}/core/kiss_matcher/KISSMatcher.hpp")
  set(SUPER_LIO_USE_VENDOR_KISS_MATCHER ON)
endif()

set(SUPER_LIO_VENDOR_ROBIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/robin")
set(SUPER_LIO_USE_VENDOR_ROBIN OFF)
if(EXISTS "${SUPER_LIO_VENDOR_ROBIN_DIR}/include/robin/robin.hpp")
  set(SUPER_LIO_USE_VENDOR_ROBIN ON)
endif()

if(NOT SUPER_LIO_USE_VENDOR_KISS_MATCHER)
  find_package(kiss_matcher QUIET)
endif()

if(TARGET kiss_matcher::kiss_matcher_core)
  message(STATUS "kiss_matcher found, enabling KISS-Matcher relocalization")
elseif(SUPER_LIO_USE_VENDOR_KISS_MATCHER)
  message(STATUS "vendored kiss_matcher found, enabling KISS-Matcher relocalization")
else()
  message(STATUS "kiss_matcher not found, KISS-Matcher relocalization disabled")
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

include_directories(
  include
  3rdparty
  3rdparty/ground_segmentation/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${basic_INCLUDE_DIRS}
  ${livox_ros_driver2_INCLUDE_DIRS}
)

file(GLOB_RECURSE LIO_LIB_SRC CONFIGURE_DEPENDS "src/lio/*.cpp" "src/ros/*.cpp")
file(GLOB_RECURSE LIO_LIB_HEADERS CONFIGURE_DEPENDS "include/lio/*.hpp" "include/lio/*.h")

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

add_library(lio ${LIO_LIB_SRC} ${LIO_LIB_HEADERS})
target_compile_features(lio PUBLIC cxx_std_20)

target_link_libraries(lio PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  glog
  Threads::Threads
  ${cpp_typesupport_target}
  gtsam
  ${rclcpp_LIBRARIES}
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

if(TARGET kiss_matcher::kiss_matcher_core)
  target_compile_definitions(lio PUBLIC HAS_KISS_MATCHER)
  target_link_libraries(lio PUBLIC kiss_matcher::kiss_matcher_core)
endif()

if(SUPER_LIO_USE_VENDOR_FAST_GICP)
  target_compile_definitions(lio PUBLIC HAS_FAST_GICP)
  target_link_libraries(lio PUBLIC fast_gicp_vendor)
elseif(fast_gicp_FOUND)
  target_compile_definitions(lio PUBLIC HAS_FAST_GICP)
  if(TARGET fast_gicp::fast_gicp)
    target_link_libraries(lio PUBLIC fast_gicp::fast_gicp)
  elseif(TARGET fast_gicp)
    target_link_libraries(lio PUBLIC fast_gicp)
  else()
    target_include_directories(lio PUBLIC ${fast_gicp_INCLUDE_DIRS})
    target_link_libraries(lio PUBLIC ${fast_gicp_LIBRARIES})
  endif()
endif()

target_include_directories(lio PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${livox_ros_driver2_INCLUDE_DIRS}
  ${basic_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

add_executable(super_lio_node src/apps/super_lio_node.cpp)
target_link_libraries(super_lio_node
  lio
  ${cpp_typesupport_target}
)

add_executable(relocation_node src/apps/relocation_node.cpp)
target_link_libraries(relocation_node
  lio
  ${cpp_typesupport_target}
)

install(TARGETS
  lio
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
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  relocation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  3rdparty/
  DESTINATION include/${PROJECT_NAME}
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

ament_package()

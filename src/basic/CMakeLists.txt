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
find_package(Glog REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
find_package(Threads REQUIRED)


set(BASIC_CORE_SOURCES
  src/Manifold.cpp
)

set(BASIC_LEGACY_SOURCES
  src/logs.cpp
)

# Keep the public headers in the target for IDE visibility and installation
# dependency tracking, while keeping the compiled source boundary explicit.
set(BASIC_HEADERS
  include/basic/Manifold.h
  include/basic/alias.h
  include/basic/ds.h
  include/basic/logs.h
  include/basic/math.h
  include/basic/buffer/LatestOnlyBuffer.hpp
  include/basic/buffer/MultiSourceLatestBuffer.hpp
  include/basic/buffer/RingBuffer.hpp
)

add_library(basic_core STATIC
  ${BASIC_CORE_SOURCES}
  include/basic/core_alias.h
  include/basic/Manifold.h
)

target_link_libraries(basic_core
  PUBLIC
  Eigen3::Eigen
  Threads::Threads
)

target_include_directories(basic_core
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  ${BASIC_LEGACY_SOURCES}
  ${BASIC_HEADERS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  basic_core
  ${PCL_LIBRARIES}
  gflags
  glog
  Threads::Threads
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
  TARGETS basic_core ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} basic_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_manifold test/test_manifold.cpp)
  target_link_libraries(test_manifold ${PROJECT_NAME})

  ament_add_gtest(test_ring_buffer test/test_ring_buffer.cpp)
  target_link_libraries(test_ring_buffer ${PROJECT_NAME})
endif()

ament_package()

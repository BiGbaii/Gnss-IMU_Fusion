cmake_minimum_required(VERSION 3.5)
project(flash3d_gnss_imu_fusion)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindGlog.cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(Glog REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
rclcpp
Eigen3
#Glog
sensor_msgs
nav_msgs
geometry_msgs
)

# Add GeographicLib.
add_subdirectory(third_party/GeographicLib)


add_executable(flash3d_GINS src/main.cpp src/flash3d_eskf.cpp src/gins.cpp ) 

target_include_directories(flash3d_GINS PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  third_party/GeographicLib/include
)
target_include_directories(flash3d_GINS PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(flash3d_GINS
  ${GLOG_LIBRARIES}
  Eigen3::Eigen
  libGeographiccc
)

ament_target_dependencies(flash3d_GINS ${dependencies})


install(TARGETS flash3d_GINS
    DESTINATION lib/${PROJECT_NAME}
  )

ament_package()

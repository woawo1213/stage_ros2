cmake_minimum_required(VERSION 3.5)
project(stage_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stage REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

ament_package()

# Declare a cpp executable
add_executable(stageros src/stageros.cpp)
set(${PROJECT_NAME}_extra_libs "")
if(UNIX AND NOT APPLE)
  set(${PROJECT_NAME}_extra_libs dl)
endif()
ament_target_dependencies(stageros
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  stage
)
target_include_directories(stageros PRIVATE
  ${Boost_INCLUDE_DIRS}
  ${STAGE_INCLUDE_DIRS}
)
target_link_libraries(stageros
  ${Boost_LIBRARIES}
  ${STAGE_LIBRARIES}
  ${${PROJECT_NAME}_extra_libs}
)

## Install
install(TARGETS stageros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch map resource rviz src world
  DESTINATION share/${PROJECT_NAME})
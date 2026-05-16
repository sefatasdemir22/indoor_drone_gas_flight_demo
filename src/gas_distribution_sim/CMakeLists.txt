cmake_minimum_required(VERSION 3.8)
project(gas_distribution_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)

add_library(gas_field_plugin SHARED
  src/gas_field_plugin.cpp
)

target_include_directories(gas_field_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gas_field_plugin
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_ros
  gazebo_dev
)

install(TARGETS gas_field_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# DÜZELTME BURADA: 'launch' klasörünü ekledik
install(DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

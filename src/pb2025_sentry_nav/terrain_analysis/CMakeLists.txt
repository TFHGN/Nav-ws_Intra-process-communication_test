cmake_minimum_required(VERSION 3.5)
project(terrain_analysis)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(terrain_analysis SHARED
  src/terrain_analysis_new.cpp
)
# Include directories
target_include_directories(terrain_analysis
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(terrain_analysis
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  pcl_ros
  rclcpp_components
  pcl_conversions
)

rclcpp_components_register_node(terrain_analysis
  PLUGIN "terrain_analysis::TerrainAnalysisNode"
  EXECUTABLE terrain_analysis_node
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_terrain_analysis)
ament_export_include_directories(include)
install(TARGETS terrain_analysis
  EXPORT export_terrain_analysis
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

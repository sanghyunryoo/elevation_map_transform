cmake_minimum_required(VERSION 3.8)
project(elevation_map_transform)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 필수 패키지 추가
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(PCL REQUIRED)

add_executable(elevation_map_transform_node src/elevation_map_transform_node.cpp)
ament_target_dependencies(elevation_map_transform_node 
  rclcpp std_msgs tf2_ros tf2_geometry_msgs grid_map_ros grid_map_msgs PCL
)

install(TARGETS
  elevation_map_transform_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

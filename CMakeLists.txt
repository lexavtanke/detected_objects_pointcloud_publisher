
cmake_minimum_required(VERSION 3.14)
project(detected_objects_pointcloud_publisher)

find_package(autoware_cmake REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
autoware_package()

include_directories(
  include
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_COMMON_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${PCL_DIR}
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ament_auto_add_library(detected_objects_pointcloud_pub_node SHARED
#   src/detected_objects_pointcloud_publisher_node.cpp
#   # include/detected_objects_pointcloud_publisher/detected_objects_pointcloud_pub_node.hpp
# )



# rclcpp_components_register_node(detected_objects_pointcloud_pub_node
#   PLUGIN "detected_objects_pointcloud_publisher::DetectedObjectsPointcloudPubNode"
#   EXECUTABLE detected_objects_pointcloud_pub
# )

ament_auto_add_executable(detected_objects_pointcloud_pub src/detected_objects_pointcloud_publisher_node.cpp)

target_link_libraries(detected_objects_pointcloud_pub
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
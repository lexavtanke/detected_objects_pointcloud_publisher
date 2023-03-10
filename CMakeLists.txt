
cmake_minimum_required(VERSION 3.14)
project(detected_objects_pointcloud_publisher)

find_package(autoware_cmake REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters search)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
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

add_library(percepted_objects_pointcloud_publisher SHARED
src/percepted_objects_pointcloud_publisher.cpp
src/detected_objects_pointcloud_publisher_node.cpp)
target_compile_definitions(percepted_objects_pointcloud_publisher
  PRIVATE "MINIMAL_COMPOSITION_DLL")

ament_target_dependencies(percepted_objects_pointcloud_publisher rclcpp sensor_msgs autoware_auto_perception_msgs
tier4_autoware_utils
pcl_conversions
tf2_ros
tf2_sensor_msgs)


# rclcpp_components_register_node(detected_objects_pointcloud_pub_node
#   PLUGIN "detected_objects_pointcloud_publisher::DetectedObjectsPointcloudPubNode"
#   EXECUTABLE detected_objects_pointcloud_pub
# )

ament_auto_add_executable(detected_objects_pointcloud_pub src/detected_objects_pointcloud_publisher_node.cpp)
target_link_libraries(detected_objects_pointcloud_pub percepted_objects_pointcloud_publisher)

# ament_target_dependencies (detected_objects_pointcloud_pub percepted_objects_pointcloud_publisher rclcBpp)
# ament_auto_add_executable(tracked_objects_pointcloud_pub src/tracked_objects_pointcloud_publisher_node.cpp)
# ament_auto_add_executable(predicted_objects_pointcloud_pub src/predicted_objects_pointcloud_publisher_node.cpp)
# ament_auto_add_executable(base_detected_objects_pointcloud_pub src/detected_objects_pointcloud_publisher.cpp)

target_include_directories(detected_objects_pointcloud_pub
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(detected_objects_pointcloud_pub
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
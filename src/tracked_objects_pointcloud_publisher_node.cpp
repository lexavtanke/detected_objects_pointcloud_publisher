
#include "detected_objects_pointcloud_publisher/detected_objects_pointcloud_publisher.hpp"
#include <pcl/kdtree/kdtree.h>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("tracked_objects_pointcloud_publisher package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackedObjectsPointcloudPublisher>());
  rclcpp::shutdown();
  return 0;
}

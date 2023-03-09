

#include "detected_objects_pointcloud_publisher/detected_objects_pointcloud_publisher.hpp"
#include <pcl/kdtree/kdtree.h>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("predicted_objects_pointcloud_publisher package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PredictedObjectsPointcloudPublisher>());
  rclcpp::shutdown();
  return 0;
}

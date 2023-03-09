

#include "detected_objects_pointcloud_publisher/detected_objects_pointcloud_publisher_node.hpp"

#include <rclcpp/rclcpp.hpp>

namespace detected_objects_pointcloud_publisher 
{
  void DetectedObjectsPointcloudPublisher::objectsCallback(const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_objs_msg)
  {

    // Transform to pointcloud frame
    autoware_auto_perception_msgs::msg::DetectedObjects transformed_objects;
    if (!transformObjects(
          *input_objs_msg, pointcloud_frame_id_, tf_buffer_,
          transformed_objects)) {
      // objects_pub_->publish(*input_objects);
      return;
    }
    
    objects_frame_id_ = transformed_objects.header.frame_id;

    objs_buffer.clear();
    for (const auto & object : transformed_objects.objects)
    {
      std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> labels = object.classification;
      object_info info = {object.shape, object.kinematics.pose_with_covariance.pose, object.classification};
      objs_buffer.push_back(info);
    }
    // RCLCPP_INFO(this->get_logger(), "Update objects buffer");
  }


  int main(int argc, char ** argv)
  {
    (void) argc;
    (void) argv;

    printf("detected_objects_pointcloud_publisher package\n");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectedObjectsPointcloudPublisher>());
    rclcpp::shutdown();
    return 0;
  }

} // namespace detected_objects_pointcloud_publisher

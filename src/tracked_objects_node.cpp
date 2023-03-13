# include "detected_objects_pointcloud_publisher.hpp"
# include <rclcpp/rclcpp.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_object.hpp>


class TrackedObjectsPointcloudPublisher : public PerceptedObjectsPointcloudPublisher<autoware_auto_perception_msgs::msg::TrackedObjects> {

  private:
  void objectsCallback(const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr input_objs_msg) override
  {
    RCLCPP_INFO(this->get_logger(), "New objects msg timestamp is '%d'.'%d'", input_objs_msg->header.stamp.sec, input_objs_msg->header.stamp.nanosec);
    objects_timestamp_ = input_objs_msg->header.stamp;
    point_color_ ={5, 5, 255}; // blue color
    // Transform to pointcloud frame
    autoware_auto_perception_msgs::msg::TrackedObjects transformed_objects;
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

};


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("Detected_objects_pointcloud_publisher package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackedObjectsPointcloudPublisher>());
  rclcpp::shutdown();
  return 0;
}

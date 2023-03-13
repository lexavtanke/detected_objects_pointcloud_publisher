# include "detected_objects_pointcloud_publisher.hpp"
# include <rclcpp/rclcpp.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>

class PredictedObjectsPointcloudPublisher : public PerceptedObjectsPointcloudPublisher<autoware_auto_perception_msgs::msg::PredictedObjects> {
  public:
    bool transformObjects(
    const autoware_auto_perception_msgs::msg::PredictedObjects & input_msg, const std::string & target_frame_id, const tf2_ros::Buffer & tf_buffer,
    autoware_auto_perception_msgs::msg::PredictedObjects & output_msg)
  {
    output_msg = input_msg;

    // transform to world coordinate
    if (input_msg.header.frame_id != target_frame_id) {
      output_msg.header.frame_id = target_frame_id;
      tf2::Transform tf_target2objects_world;
      tf2::Transform tf_target2objects;
      tf2::Transform tf_objects_world2objects;
      {
        const auto ros_target2objects_world = getTransform(
          tf_buffer, input_msg.header.frame_id, target_frame_id, input_msg.header.stamp);
        if (!ros_target2objects_world) {
          return false;
        }
        tf2::fromMsg(*ros_target2objects_world, tf_target2objects_world);
      }
      for (auto & object : output_msg.objects) {
        tf2::fromMsg(object.kinematics.initial_pose_with_covariance.pose, tf_objects_world2objects);
        tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
        tf2::toMsg(tf_target2objects, object.kinematics.initial_pose_with_covariance.pose);
      }
    }
    return true;
  }

  private:
  void objectsCallback(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr input_objs_msg) override
  {
    RCLCPP_INFO(this->get_logger(), "New objects msg timestamp is '%d'.'%d'", input_objs_msg->header.stamp.sec, input_objs_msg->header.stamp.nanosec);
    objects_timestamp_ = input_objs_msg->header.stamp;
    point_color_ ={5, 255, 5}; // green color
    // RCLCPP_INFO(this->get_logger(), "Befor transform objects frame is '%s'", input_objs_msg->header.frame_id.c_str());
    // RCLCPP_INFO(this->get_logger(), "First object X is '%f' Y is '%f'", 
    // input_objs_msg->objects.at(0).kinematics.initial_pose_with_covariance.pose.position.x,
    // input_objs_msg->objects.at(0).kinematics.initial_pose_with_covariance.pose.position.y);
    // Transform to pointcloud frame
    autoware_auto_perception_msgs::msg::PredictedObjects transformed_objects;
    if (!transformObjects(
          *input_objs_msg, pointcloud_frame_id_, tf_buffer_,
          transformed_objects)) {
      // objects_pub_->publish(*input_objects);
      RCLCPP_INFO(this->get_logger(), "Did NOT transform objects");
      return;
    }
    // RCLCPP_INFO(this->get_logger(), "Transform DONE");
    // RCLCPP_INFO(this->get_logger(), "After transform objects frame is '%s'", transformed_objects.header.frame_id.c_str());
    // RCLCPP_INFO(this->get_logger(), "First object X is '%f' Y is '%f'", 
    // transformed_objects.objects.at(0).kinematics.initial_pose_with_covariance.pose.position.x,
    // transformed_objects.objects.at(0).kinematics.initial_pose_with_covariance.pose.position.y);

    objects_frame_id_ = transformed_objects.header.frame_id;
    objs_buffer.clear();
    for (const auto & object : transformed_objects.objects)
    {
      std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> labels = object.classification;
      object_info info = {object.shape, object.kinematics.initial_pose_with_covariance.pose, object.classification};
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
  rclcpp::spin(std::make_shared<PredictedObjectsPointcloudPublisher>());
  rclcpp::shutdown();
  return 0;
}

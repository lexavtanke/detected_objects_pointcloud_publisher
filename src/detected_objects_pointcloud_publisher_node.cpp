#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/detected_object.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>


class DetectedObjectsPointcloudPublisher : public rclcpp::Node
{
public:
  DetectedObjectsPointcloudPublisher() : Node ("detected_objects_poincloud_publisher"),  
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud", 10);  
    pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input/pointcloud", 
      rclcpp::SensorDataQoS(), 
      std::bind(&DetectedObjectsPointcloudPublisher::pointCloudCallback, 
      this, 
      std::placeholders::_1));
    detected_objects_subscription_ = this->create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
      "input/detected_objects", 
      rclcpp::QoS{1}, 
      std::bind(&DetectedObjectsPointcloudPublisher::objectsCallback, 
      this, 
      std::placeholders::_1));

    autoware_auto_perception_msgs::msg::DetectedObjects::SharedPtr objs_buffer;

    RCLCPP_INFO(this->get_logger(), "Hello from detected_objects_pointcloud_publisher constructor\n");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2 input_pointcloud_msg) 
  {
    sensor_msgs::msg::PointCloud2 transformed_pointcloud;
    if (objects_frame_id_ != "" && input_pointcloud_msg.header.frame_id != objects_frame_id_) 
    {
      geometry_msgs::msg::TransformStamped transform;
      transform = tf_buffer_.lookupTransform(
      input_pointcloud_msg.header.frame_id, objects_frame_id_, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));

      tf2::doTransform(input_pointcloud_msg, transformed_pointcloud, transform);
      
    } else {
      transformed_pointcloud = input_pointcloud_msg;
    }
    
    pcl::PCLPointCloud2 input_pcl_cloud;
    pcl_conversions::toPCL(transformed_pointcloud, input_pcl_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(input_pcl_cloud, *temp_cloud);
    
    // Create a new point cloud with RGB color information and copy data from input cloudb
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*temp_cloud, *colored_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      
  
  }

  void objectsCallback(const autoware_auto_perception_msgs::msg::DetectedObjects::SharedPtr input_detected_objs_msg)
  {
    objects_frame_id_ = input_detected_objs_msg->header.frame_id;
    objs_buffer.clear();
    for (size_t i = 0; i != input_detected_objs_msg->objects.size(); i++)
    {
      objs_buffer.push_back(input_detected_objs_msg->objects.at(i));
    }
    // RCLCPP_INFO(this->get_logger(), "Update objects buffer");
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr detected_objects_subscription_;

  std::vector<autoware_auto_perception_msgs::msg::DetectedObject> objs_buffer;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string objects_frame_id_;


};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world detected_objects_pointcloud_publisher package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectedObjectsPointcloudPublisher>());
  rclcpp::shutdown();
  return 0;
}

#include <cstdio>
// #include <vector>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/detected_object.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class DetectedObjectsPointcloudPublisher : public rclcpp::Node
{
public:
  DetectedObjectsPointcloudPublisher() : Node ("detected_objects_poincloud_publisher"),  
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter("min_x", 0.0f);
    this->declare_parameter("max_x", 1.0f);
    this->declare_parameter("min_y", 0.0f);
    this->declare_parameter("max_y", 1.0f);
    this->declare_parameter("min_z", 0.0f);
    this->declare_parameter("max_z", 1.0f);
    this->declare_parameter("trans_x", 1.0f);
    this->declare_parameter("trans_y", 1.0f);
    this->declare_parameter("trans_z", 1.0f);
    this->declare_parameter("filter_axis", "x");
    this->declare_parameter("filter_min", 0.0f);
    this->declare_parameter("filter_max", 2.0f);

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud", 10);  
    pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input/pointcloud", 
      10, 
      std::bind(&DetectedObjectsPointcloudPublisher::pointCloudCallback, 
      this, 
      std::placeholders::_1));
    detected_objects_subscription_ = this->create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
      "input/detected_objects", 
      10, 
      std::bind(&DetectedObjectsPointcloudPublisher::objectsCallback, 
      this, 
      std::placeholders::_1));

    autoware_auto_perception_msgs::msg::DetectedObjects::SharedPtr objs_buffer;

    RCLCPP_INFO(this->get_logger(), "Hello from detected_objects_pointcloud_publisher constructor\n");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg) 
  {
    
    pcl::PCLPointCloud2 input_pcl_cloud;
    pcl_conversions::toPCL(*input_pointcloud_msg, input_pcl_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(input_pcl_cloud, *temp_cloud);
    
    // Create a new point cloud with RGB color information and copy data from input cloudb
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*temp_cloud, *colored_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    

    // define box filter size
    // float max_x = this->get_parameter("max_x").get_parameter_value().get<float>();
    // float min_x = this->get_parameter("min_x").get_parameter_value().get<float>();
    // float max_y = this->get_parameter("max_y").get_parameter_value().get<float>();
    // float min_y = this->get_parameter("min_y").get_parameter_value().get<float>();
    // float max_z = this->get_parameter("max_z").get_parameter_value().get<float>();
    // float min_z = this->get_parameter("min_z").get_parameter_value().get<float>();
    // Eigen::Vector4f min_pt (min_x, min_y, min_z, 0.0f);
    // Eigen::Vector4f max_pt (max_x, max_y, max_z, 0.0f);

    // cropBoxFilter.setMin(min_pt);
    // cropBoxFilter.setMax(max_pt);

    // float trans_x = this->get_parameter("trans_x").get_parameter_value().get<float>();
    // float trans_y = this->get_parameter("trans_y").get_parameter_value().get<float>();
    // float trans_z = this->get_parameter("trans_z").get_parameter_value().get<float>();
    // Eigen::Vector3f box_translation(trans_x, trans_y, trans_z);
    // cropBoxFilter.setTranslation(box_translation);
      
    if (objs_buffer.size() > 0) {
      for (auto object : objs_buffer) {

    
        pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;
        pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter (true);
        cropBoxFilter.setInputCloud(colored_cloud);
        float trans_x = object.kinematics.pose_with_covariance.pose.position.x;
        float trans_y = object.kinematics.pose_with_covariance.pose.position.y;
        float trans_z = object.kinematics.pose_with_covariance.pose.position.z;
        // Eigen::Vector3f boxlation(trans_x, trans_y, trans_z);
        // cropBoxFilter.setTranslation(box_translation);

        // float max_x =  + object.shape.dimensions.x / 2.0;
        // float min_x =  - object.shape.dimensions.x / 2.0;
        // float max_y =  + object.shape.dimensions.y / 2.0;
        // float min_y =  - object.shape.dimensions.y / 2.0;
        // float max_z =  + object.shape.dimensions.z / 2.0;
        // float min_z =  - object.shape.dimensions.z / 2.0; 
        float max_x = trans_x + object.shape.dimensions.x / 2.0;
        float min_x = trans_x - object.shape.dimensions.x / 2.0;
        float max_y = trans_y + object.shape.dimensions.y / 2.0;
        float min_y = trans_y - object.shape.dimensions.y / 2.0;
        float max_z = trans_z + object.shape.dimensions.z / 2.0;
        float min_z = trans_z - object.shape.dimensions.z / 2.0; 

        Eigen::Vector4f min_pt (min_x, min_y, min_z, 0.0f);
        Eigen::Vector4f max_pt (max_x, max_y, max_z, 0.0f);
        cropBoxFilter.setMin(min_pt);
        cropBoxFilter.setMax(max_pt);
        cropBoxFilter.filter(filtered_cloud);

        // Define a custom color for the box polygons
        const uint8_t r = 0, g = 0, b = 255;

        for (auto cloud_it = filtered_cloud.begin(); cloud_it != filtered_cloud.end(); ++cloud_it)
        {
          cloud_it->r = r;
          cloud_it->g = g;
          cloud_it->b = b;
        }
        
        *out_cloud += filtered_cloud;

      }
      
    }
    // // cropBoxFilter.setNegative(true);



    // std::vector<int> indices;
    // cropBoxFilter.filter(indices);

    // std::string filter_axis = this->get_parameter("filter_axis").get_parameter_value().get<std::string>();
    // float filter_max = this->get_parameter("filter_max").get_parameter_value().get<float>();
    // float filter_min = this->get_parameter("filter_min").get_parameter_value().get<float>();
    //     // Create the filtering object
    // pcl::PassThrough<pcl::PointXYZRGB> pass;
    // pass.setInputCloud (colored_cloud);
    // pass.setFilterFieldName (filter_axis);
    // pass.setFilterLimits (filter_min, filter_max);
    // //pass.setFilterLimitsNegative (true);
    // pass.filter(indices);
    // pass.filter(out_cloud);
    
    // Define a custom color for the box polygons
    // const uint8_t r = 255, g = 0, b = 0;
    
    // for (size_t i = 0; i < indices.size(); i++) {
    //   colored_cloud->points[i].r = r;
    //   colored_cloud->points[i].g = g;
    //   colored_cloud->points[i].b = b;
    // }

    sensor_msgs::msg::PointCloud2::SharedPtr output_pointcloud_msg_ptr( new sensor_msgs::msg::PointCloud2);
    // pcl::toROSMsg(*colored_cloud, *output_pointcloud_msg_ptr);
    
    pcl::toROSMsg(*out_cloud, *output_pointcloud_msg_ptr);
    output_pointcloud_msg_ptr->header = input_pointcloud_msg->header;

    // TODO(lexavtanke) get pointcloud in frame base link and detected objects in frame map
    // change pointcloud frame to map 
    // update color of the points and remove all points outside deceted objects 

    
    RCLCPP_INFO(this->get_logger(), "Publishing pointcloud");
    publisher_->publish(*output_pointcloud_msg_ptr);
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

  // void filterPointcloud(const pcl::PointCloud<pcl::)
  // {

  // }

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

#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_object.hpp>
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/passthrough.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

using Shape = autoware_auto_perception_msgs::msg::Shape;

  // struct for creating objects buffer 
struct object_info
{
  Shape shape;
  geometry_msgs::msg::Point position; 
  geometry_msgs::msg::Quaternion orientation;
  std::vector<autoware_auto_perception_msgs::msg::ObjectClassification>  classification;
};


template <typename MsgT>
class PerceptedObjectsPointcloudPublisher : public rclcpp::Node
{
public:
  PerceptedObjectsPointcloudPublisher() : Node ("perception_objects_poincloud_publisher"),  
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud", 10);  
    pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input/pointcloud", 
      rclcpp::SensorDataQoS(), 
      std::bind(&PerceptedObjectsPointcloudPublisher::pointCloudCallback, 
      this, 
      std::placeholders::_1));
    percepted_objects_subscription_ = this->create_subscription<MsgT>(
      "input/detected_objects", 
      rclcpp::QoS{1}, 
      std::bind(&PerceptedObjectsPointcloudPublisher::objectsCallback, 
      this, 
      std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Hello from percepted_objects_pointcloud_publisher constructor\n");
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  typename rclcpp::Subscription<MsgT>::SharedPtr percepted_objects_subscription_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<object_info> objs_buffer;
  std::string objects_frame_id_;

private:

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2 input_pointcloud_msg) 
  {
    pcl::PointXYZ minPt, maxPt;
    pcl::PointCloud<pcl::PointXYZ> measured_cloud;
    pcl::fromROSMsg(input_pointcloud_msg, measured_cloud);
    pcl::getMinMax3D(measured_cloud, minPt, maxPt);
    RCLCPP_INFO(this->get_logger(), "before translation max X is '%f' max Y is '%f'", maxPt.x, maxPt.y);
    RCLCPP_INFO(this->get_logger(), "before translation min X is '%f' min Y is '%f'", minPt.x, minPt.y);

    // do transform if needed
    sensor_msgs::msg::PointCloud2 transformed_pointcloud;
    if (objects_frame_id_ != "" && input_pointcloud_msg.header.frame_id != objects_frame_id_) 
    {
      geometry_msgs::msg::TransformStamped transform;
      transform = tf_buffer_.lookupTransform(
      objects_frame_id_, input_pointcloud_msg.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.5));

      tf2::doTransform(input_pointcloud_msg, transformed_pointcloud, transform);
      RCLCPP_INFO(this->get_logger(), "Transform done");
      pcl::PointCloud<pcl::PointXYZ> transformed_measured_cloud;
      pcl::fromROSMsg(transformed_pointcloud, transformed_measured_cloud);
      pcl::getMinMax3D(transformed_measured_cloud, minPt, maxPt);
      RCLCPP_INFO(this->get_logger(), "after translation max X is '%f' max Y is '%f'", maxPt.x, maxPt.y);
      RCLCPP_INFO(this->get_logger(), "after translation min X is '%f' min Y is '%f'", minPt.x, minPt.y);
      
    } else {
      transformed_pointcloud = input_pointcloud_msg;
      RCLCPP_INFO(this->get_logger(), "Pass without transform");
    }
    
    // convert to pcl pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(transformed_pointcloud, *temp_cloud);

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    // pcl::fromROSMsg(transformed_pointcloud, *colored_cloud);
    
    // Create a new point cloud with RGB color information and copy data from input cloudb
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*temp_cloud, *colored_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      
    if (objs_buffer.size() > 0) {
      RCLCPP_INFO(this->get_logger(), "Filtering pointcloud");
      for (auto object : objs_buffer) {
  
        if (object.shape.type == Shape::BOUNDING_BOX || object.shape.type == Shape::CYLINDER )
        {
          pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;
          pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter (true);
          cropBoxFilter.setInputCloud(colored_cloud);
          
          // float trans_x = object.kinematics.pose_with_covariance.pose.position.x;
          // float trans_y = object.kinematics.pose_with_covariance.pose.position.y;
          // float trans_z = object.kinematics.pose_with_covariance.pose.position.z;

          float trans_x = object.position.x;
          float trans_y = object.position.y;
          float trans_z = object.position.z;
          Eigen::Vector3f translation (trans_x, trans_y, trans_z);
          cropBoxFilter.setTranslation(translation);

          float rot_x = object.orientation.x;
          float rot_y = object.orientation.y;
          float rot_z = object.orientation.z;
          Eigen::Vector3f rotation(rot_x, rot_y, rot_z);
          cropBoxFilter.setRotation(rotation);

          float max_x =   object.shape.dimensions.x / 2.0;
          float min_x = - object.shape.dimensions.x / 2.0;
          float max_y =   object.shape.dimensions.y / 2.0;
          float min_y = - object.shape.dimensions.y / 2.0;
          float max_z =   object.shape.dimensions.z / 2.0;
          float min_z = - object.shape.dimensions.z / 2.0; 

          Eigen::Vector4f min_pt (min_x, min_y, min_z, 0.0f);
          Eigen::Vector4f max_pt (max_x, max_y, max_z, 0.0f);
          cropBoxFilter.setMin(min_pt);
          cropBoxFilter.setMax(max_pt);
          cropBoxFilter.filter(filtered_cloud);

          // Define a custom color for the box polygons
          const uint8_t r = 30, g = 44, b = 255;

          for (auto cloud_it = filtered_cloud.begin(); cloud_it != filtered_cloud.end(); ++cloud_it)
          {
            cloud_it->r = r;
            cloud_it->g = g;
            cloud_it->b = b;
          }

          *out_cloud += filtered_cloud;

        } else if (object.shape.type == Shape::POLYGON) 
        {
          filterPolygon(colored_cloud, out_cloud, object);
        }

      }
      
    }


    sensor_msgs::msg::PointCloud2::SharedPtr output_pointcloud_msg_ptr( new sensor_msgs::msg::PointCloud2);
    // pcl::toROSMsg(*colored_cloud, *output_pointcloud_msg_ptr); 
    pcl::toROSMsg(*out_cloud, *output_pointcloud_msg_ptr);
    // *output_pointcloud_msg_ptr = transformed_pointcloud;

    output_pointcloud_msg_ptr->header = input_pointcloud_msg.header;
    output_pointcloud_msg_ptr->header.frame_id = objects_frame_id_;

    // TODO(lexavtanke) get pointcloud in frame base link and detected objects in frame map
    // change pointcloud frame to map 
    // update color of the points and remove all points outside deceted objects 

    
    RCLCPP_INFO(this->get_logger(), "Publishing pointcloud");
    publisher_->publish(*output_pointcloud_msg_ptr);
  }


  void filterPolygon(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in_cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr out_cloud, const object_info &object)
  {
        pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;
        pcl::CropHull<pcl::PointXYZRGB> crop_hull_filter;
        crop_hull_filter.setInputCloud(in_cloud);
        
        // float trans_x = object.kinematics.pose_with_covariance.pose.position.x;
        // float trans_y = object.kinematics.pose_with_covariance.pose.position.y;
        // float trans_z = object.kinematics.pose_with_covariance.pose.position.z;

        float trans_x = object.position.x;
        float trans_y = object.position.y;
        float trans_z = object.position.z;
        Eigen::Vector3f translation (trans_x, trans_y, trans_z);

        float rot_x = object.orientation.x;
        float rot_y = object.orientation.y;
        float rot_z = object.orientation.z;
        Eigen::Quaternionf rotation(rot_x, rot_y, rot_z, 1.0);

        // create filtering pointcloud from object polygon

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (size_t i = 0; i < object.shape.footprint.points.size(); ++i) 
        {
          pcl::PointXYZRGB point;
          point.x = object.shape.footprint.points.at(i).x;
          point.y = object.shape.footprint.points.at(i).y;
          point.z = - object.shape.dimensions.z / 2.0;
          hull_cloud->push_back(point);  
          point.x = object.shape.footprint.points.at(i).x;
          point.y = object.shape.footprint.points.at(i).y;
          point.z = object.shape.dimensions.z / 2.0;
          hull_cloud->push_back(point);
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_hull_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::transformPointCloud(hull_cloud, trans_hull_cloud, translation, rotation);      
        crop_hull_filter.setHullCloud(hull_cloud->makeShared());

        crop_hull_filter.filter(filtered_cloud);

        // Define a custom color for the box polygons
        const uint8_t r = 30, g = 44, b = 255;

        for (auto cloud_it = filtered_cloud.begin(); cloud_it != filtered_cloud.end(); ++cloud_it)
        {
          cloud_it->r = r;
          cloud_it->g = g;
          cloud_it->b = b;
        }

        *out_cloud += filtered_cloud;
  }

 virtual void objectsCallback(typename MsgT::ConstSharedPtr msg) = 0;

  // void objectsCallback(const autoware_auto_perception_msgs::msg::DetectedObjects::SharedPtr input_detected_objs_msg)
  // {
  //   objects_frame_id_ = input_detected_objs_msg->header.frame_id;
  //   objs_buffer.clear();
  //   for (auto object : input_detected_objs_msg.)
  //   {
  //     objs_buffer.push_back({});
  //   }
  //   // RCLCPP_INFO(this->get_logger(), "Update objects buffer");
  // }

};

class DetectedObjectsPointcloudPublisher : public PerceptedObjectsPointcloudPublisher<autoware_auto_perception_msgs::msg::DetectedObjects> {

  private:
  void objectsCallback(const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_detected_objs_msg) override
  {
    objects_frame_id_ = input_detected_objs_msg->header.frame_id;
    objs_buffer.clear();
    for (const auto & object : input_detected_objs_msg->objects)
    {
      std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> labels = object.classification;
      object_info info = {object.shape, object.kinematics.pose_with_covariance.pose.position,
      object.kinematics.pose_with_covariance.pose.orientation, object.classification};
      objs_buffer.push_back(info);
    }
    // RCLCPP_INFO(this->get_logger(), "Update objects buffer");
  }

};

class TrackedObjectsPointcloudPublisher : public PerceptedObjectsPointcloudPublisher<autoware_auto_perception_msgs::msg::TrackedObjects> {

  private:
  void objectsCallback(const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr input_predicted_objs_msg) override
  {
    objects_frame_id_ = input_predicted_objs_msg->header.frame_id;
    objs_buffer.clear();
    for (const auto & object : input_predicted_objs_msg->objects)
    {
      std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> labels = object.classification;
      object_info info = {object.shape, object.kinematics.pose_with_covariance.pose.position,
      object.kinematics.pose_with_covariance.pose.orientation, object.classification};
      objs_buffer.push_back(info);
    }
    // RCLCPP_INFO(this->get_logger(), "Update objects buffer");
  }

};


class PredictedObjectsPointcloudPublisher : public PerceptedObjectsPointcloudPublisher<autoware_auto_perception_msgs::msg::PredictedObjects> {

  private:
  void objectsCallback(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr input_predicted_objs_msg) override
  {
    objects_frame_id_ = input_predicted_objs_msg->header.frame_id;
    objs_buffer.clear();
    for (const auto & object : input_predicted_objs_msg->objects)
    {
      std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> labels = object.classification;
      object_info info = {object.shape, object.kinematics.initial_pose_with_covariance.pose.position,
      object.kinematics.initial_pose_with_covariance.pose.orientation, object.classification};
      objs_buffer.push_back(info);
    }
    // RCLCPP_INFO(this->get_logger(), "Update objects buffer");
  }

};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world detected_objects_pointcloud_publisher package\n");

  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<PredictedObjectsPointcloudPublisher>());
  // rclcpp::spin(std::make_shared<TrackedObjectsPointcloudPublisher>());
  rclcpp::spin(std::make_shared<DetectedObjectsPointcloudPublisher>());
  rclcpp::shutdown();
  return 0;
}

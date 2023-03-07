#include <cstdio>
#include <boost/optional.hpp>
#include <boost/geometry.hpp>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_object.hpp>
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/pcl_search.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/passthrough.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

using Shape = autoware_auto_perception_msgs::msg::Shape;
using Polygon2d = tier4_autoware_utils::Polygon2d;

  // struct for creating objects buffer 
struct object_info
{
  Shape shape;
  geometry_msgs::msg::Pose position; 
  std::vector<autoware_auto_perception_msgs::msg::ObjectClassification>  classification;
};

inline pcl::PointXYZRGB toPCL(const double x, const double y, const double z)
{
  pcl::PointXYZRGB pcl_point;
  pcl_point.x = x;
  pcl_point.y = y;
  pcl_point.z = z;
  return pcl_point;
}

inline pcl::PointXYZRGB toPCL(const geometry_msgs::msg::Point & point)
{
  return toPCL(point.x, point.y, point.z);
}


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

  boost::optional<geometry_msgs::msg::Transform> getTransform(
    const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
    const std::string & target_frame_id, const rclcpp::Time & time)
  {
    try {
      geometry_msgs::msg::TransformStamped self_transform_stamped;
      self_transform_stamped = tf_buffer.lookupTransform(
        target_frame_id, source_frame_id, time, rclcpp::Duration::from_seconds(0.5));
      return self_transform_stamped.transform;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("perception_utils"), ex.what());
      return boost::none;
    }
  }

  bool transformObjects(
    const MsgT & input_msg, const std::string & target_frame_id, const tf2_ros::Buffer & tf_buffer,
    MsgT & output_msg)
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
        tf2::fromMsg(object.kinematics.pose_with_covariance.pose, tf_objects_world2objects);
        tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
        tf2::toMsg(tf_target2objects, object.kinematics.pose_with_covariance.pose);
      }
    }
    return true;
  }

  std::optional<float> getMaxRadius(object_info & object)
{
  if (object.shape.type == Shape::BOUNDING_BOX || object.shape.type == Shape::CYLINDER) {
    return std::hypot(object.shape.dimensions.x * 0.5f, object.shape.dimensions.y * 0.5f);
  } else if (object.shape.type == Shape::POLYGON) {
    float max_dist = 0.0;
    for (const auto & point : object.shape.footprint.points) {
      const float dist = std::hypot(point.x, point.y);
      max_dist = max_dist < dist ? dist : max_dist;
    }
    return max_dist;
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "unknown shape type");
    return std::nullopt;
  }
}

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  typename rclcpp::Subscription<MsgT>::SharedPtr percepted_objects_subscription_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<object_info> objs_buffer;
  std::string objects_frame_id_;
  std::string pointcloud_frame_id_;

private:


  void pointCloudCallback(const sensor_msgs::msg::PointCloud2 input_pointcloud_msg) 
  {
    RCLCPP_INFO(this->get_logger(), "Get new pointcloud");
    pointcloud_frame_id_ = input_pointcloud_msg.header.frame_id;

    pcl::PointXYZ minPt, maxPt;
    pcl::PointCloud<pcl::PointXYZ> measured_cloud;
    pcl::fromROSMsg(input_pointcloud_msg, measured_cloud);
    pcl::getMinMax3D(measured_cloud, minPt, maxPt);
    
    RCLCPP_INFO(this->get_logger(), "before translation max X is '%f' max Y is '%f'", maxPt.x, maxPt.y);
    RCLCPP_INFO(this->get_logger(), "before translation min X is '%f' min Y is '%f'", minPt.x, minPt.y);
    
    // convert to pcl pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromROSMsg(transformed_pointcloud, *temp_cloud);
    pcl::fromROSMsg(input_pointcloud_msg, *temp_cloud);

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    // pcl::fromROSMsg(transformed_pointcloud, *colored_cloud);
    
    // Create a new point cloud with RGB color information and copy data from input cloudb
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*temp_cloud, *colored_cloud);

    // Create Kd-tree to search neighbor pointcloud to reduce cost.
    pcl::search::Search<pcl::PointXYZRGB>::Ptr kdtree =
    pcl::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>(false);
    kdtree->setInputCloud(colored_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      
    if (objs_buffer.size() > 0) {
      RCLCPP_INFO(this->get_logger(), "Filtering pointcloud");
      for (auto object : objs_buffer) {
        RCLCPP_INFO(this->get_logger(), "object");

        const auto search_radius = getMaxRadius(object);
        // Search neighbor pointcloud to reduce cost.
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr neighbor_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<int> indices;
        std::vector<float> distances;        
        kdtree->radiusSearch(
        toPCL(object.position.position), search_radius.value(), indices, distances);
        for (const auto & index : indices) {
          neighbor_pointcloud->push_back(colored_cloud->at(index));
        }  
        
        filterPolygon(neighbor_pointcloud, out_cloud, object);
      }     
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5, "objects buffer is empty");
      return;
    }


    sensor_msgs::msg::PointCloud2::SharedPtr output_pointcloud_msg_ptr( new sensor_msgs::msg::PointCloud2);
    // pcl::toROSMsg(*colored_cloud, *output_pointcloud_msg_ptr); 
    pcl::toROSMsg(*out_cloud, *output_pointcloud_msg_ptr);
    // *output_pointcloud_msg_ptr = transformed_pointcloud;

    output_pointcloud_msg_ptr->header = input_pointcloud_msg.header;
    output_pointcloud_msg_ptr->header.frame_id = objects_frame_id_;
    
    RCLCPP_INFO(this->get_logger(), "Publishing pointcloud");
    publisher_->publish(*output_pointcloud_msg_ptr);
  }


  void filterPolygon(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud, const object_info &object)
  {        
        pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;
        std::vector<pcl::Vertices> vertices_array;
        pcl::Vertices vertices;

        Polygon2d poly2d =
        tier4_autoware_utils::toPolygon2d(object.position, object.shape);
        if (boost::geometry::is_empty(poly2d)) return;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (size_t i = 0; i < poly2d.outer().size(); ++i) {
          vertices.vertices.emplace_back(i);
          vertices_array.emplace_back(vertices);
          hull_cloud->emplace_back(static_cast<float>(poly2d.outer().at(i).x()), 
          static_cast<float>(poly2d.outer().at(i).y()), 
          static_cast<float>(0.0));
        }

        pcl::CropHull<pcl::PointXYZRGB> crop_hull_filter;
        crop_hull_filter.setInputCloud(in_cloud);
        crop_hull_filter.setDim(2);
        crop_hull_filter.setHullIndices(vertices_array);
        crop_hull_filter.setHullCloud(hull_cloud);
        crop_hull_filter.setCropOutside(true);
        
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
  void objectsCallback(const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_objs_msg) override
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

};

class TrackedObjectsPointcloudPublisher : public PerceptedObjectsPointcloudPublisher<autoware_auto_perception_msgs::msg::TrackedObjects> {

  private:
  void objectsCallback(const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr input_objs_msg) override
  {
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
    RCLCPP_INFO(this->get_logger(), "New objects");
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
    RCLCPP_INFO(this->get_logger(), "Transform DONE");
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

  printf("hello world detected_objects_pointcloud_publisher package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PredictedObjectsPointcloudPublisher>());
  // rclcpp::spin(std::make_shared<TrackedObjectsPointcloudPublisher>());
  // rclcpp::spin(std::make_shared<DetectedObjectsPointcloudPublisher>());
  rclcpp::shutdown();
  return 0;
}

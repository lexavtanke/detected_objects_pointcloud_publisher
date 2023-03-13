#include "detected_objects_pointcloud_publisher.hpp"
#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_object.hpp>
#include <std_msgs/msg/header.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>

using TrackedObjects = autoware_auto_perception_msgs::msg::TrackedObjects;
using TrackedObject = autoware_auto_perception_msgs::msg::TrackedObject;

class SyncObjectsPointcloudPublisher: public PerceptedObjectsPointcloudPublisher<TrackedObjects>
{
public:
  SyncObjectsPointcloudPublisher(): PerceptedObjectsPointcloudPublisher<TrackedObjects>(),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  sync_(SyncPolicy(10), percepted_objects_subscription_, pointcloud_subscription_)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    percepted_objects_subscription_.subscribe(this, "/input/detected_objects", rclcpp::QoS{1}.get_rmw_qos_profile()),
    pointcloud_subscription_.subscribe(
      this, "/input/pointcloud",
      rclcpp::SensorDataQoS{}.keep_last(1).get_rmw_qos_profile());
    sync_.registerCallback(
      std::bind(&SyncObjectsPointcloudPublisher::onObjectsAndObstaclePointCloud, this, _1, _2));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud", rclcpp::SensorDataQoS()); 
  } 


private:
  message_filters::Subscriber<TrackedObjects> percepted_objects_subscription_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_subscription_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  typedef message_filters::sync_policies::ApproximateTime<
    TrackedObjects, sensor_msgs::msg::PointCloud2>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  Sync sync_;


private:
  void onObjectsAndObstaclePointCloud(
    const TrackedObjects::ConstSharedPtr & input_objs_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_pointcloud_msg)
    { 
      RCLCPP_INFO(this->get_logger(), "onObjectsAndObstaclePointCloud");
      RCLCPP_INFO(this->get_logger(), " SYNC New objects msg timestamp is '%d'.'%d'", input_objs_msg->header.stamp.sec, input_objs_msg->header.stamp.nanosec);
      point_color_ ={5, 5, 255}; // blue color
      // Transform to pointcloud frame
      autoware_auto_perception_msgs::msg::TrackedObjects transformed_objects;
      if (!transformObjects(
          *input_objs_msg, pointcloud_frame_id_, tf_buffer_,
          transformed_objects)) {
      // objects_pub_->publish(*input_objects);
        return;
      }
    
      objs_buffer.clear();
      for (const auto & object : transformed_objects.objects)
      {
        std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> labels = object.classification;
        object_info info = {object.shape, object.kinematics.pose_with_covariance.pose, object.classification};
        objs_buffer.push_back(info);
      }
      objects_frame_id_ = transformed_objects.header.frame_id;

      // std_msgs::msg::Header::stamp stamp_diff = input_pointcloud_msg->header.stamp - input_objs_msg->header.stamp;
      rclcpp::Time objects_timestamp_ = input_objs_msg->header.stamp;
      rclcpp::Time pointcloud_timestamp = input_pointcloud_msg->header.stamp;
      double dt = (pointcloud_timestamp.nanoseconds() - objects_timestamp_.nanoseconds()) / 1000000; // to milisec 
      // builtin_interfaces::msg::Time 
      RCLCPP_INFO(this->get_logger(), " SYNC Get new pointcloud msg timestamp is '%d'.'%d' difference is '%f'", input_pointcloud_msg->header.stamp.sec, input_pointcloud_msg->header.stamp.nanosec, dt);
      pointcloud_frame_id_ = input_pointcloud_msg->header.frame_id;

      // pcl::PointXYZ minPt, maxPt;
      // pcl::PointCloud<pcl::PointXYZ> measured_cloud;
      // pcl::fromROSMsg(*input_pointcloud_msg, measured_cloud);
      // pcl::getMinMax3D(measured_cloud, minPt, maxPt);
      
      // RCLCPP_INFO(this->get_logger(), "before translation max X is '%f' max Y is '%f'", maxPt.x, maxPt.y);
      // RCLCPP_INFO(this->get_logger(), "before translation min X is '%f' min Y is '%f'", minPt.x, minPt.y);
      
      // convert to pcl pointcloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::fromROSMsg(transformed_pointcloud, *temp_cloud);
      pcl::fromROSMsg(*input_pointcloud_msg, *temp_cloud);

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

      // RCLCPP_INFO(this->get_logger(), "'%ld' objects in objs_buffer", objs_buffer.size());  
      if (objs_buffer.size() > 0) {
        // RCLCPP_INFO(this->get_logger(), "Filtering pointcloud");
        for (auto object : objs_buffer) {
          // RCLCPP_INFO(this->get_logger(), "object");

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

      output_pointcloud_msg_ptr->header = input_pointcloud_msg->header;
      output_pointcloud_msg_ptr->header.frame_id = objects_frame_id_;
      
      RCLCPP_INFO(this->get_logger(), "end onObjectsAndObstaclePointCloud");
      publisher_->publish(*output_pointcloud_msg_ptr);
    }

    void objectsCallback(const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr input_objs_msg) override
  { 
    autoware_auto_perception_msgs::msg::TrackedObjects output;
    output.header = input_objs_msg->header;}

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2 input_pointcloud_msg) 
  {
    rclcpp::Time pointcloud_timestamp = input_pointcloud_msg.header.stamp;
  }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("Detected_objects_pointcloud_publisher package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncObjectsPointcloudPublisher>());
  rclcpp::shutdown();
  return 0;
}

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
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/pcl_search.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/passthrough.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace detected_objects_pointcloud_publisher 
{

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
    PerceptedObjectsPointcloudPublisher();  
      // tf_buffer_(this->get_clock()),
      // tf_listener_(tf_buffer_);

    boost::optional<geometry_msgs::msg::Transform> getTransform(
      const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
      const std::string & target_frame_id, const rclcpp::Time & time);

    bool transformObjects(
      const MsgT & input_msg, const std::string & target_frame_id, const tf2_ros::Buffer & tf_buffer,
      MsgT & output_msg);

    std::optional<float> getMaxRadius(object_info & object);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
    typename rclcpp::Subscription<MsgT>::SharedPtr percepted_objects_subscription_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::vector<object_info> objs_buffer;
    std::string objects_frame_id_;
    std::string pointcloud_frame_id_;

  private:

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2 input_pointcloud_msg);
    void filterPolygon(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud, const object_info &object);
    virtual void objectsCallback(typename MsgT::ConstSharedPtr msg) = 0;
  };

} // namespace detected_objects_pointcloud_publisher 

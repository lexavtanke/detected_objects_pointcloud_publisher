
#include "detected_objects_pointcloud_publisher/percepted_objects_pointcloud_publisher.hpp"
namespace detected_objects_pointcloud_publisher 
{
    class DetectedObjectsPointcloudPublisher : public PerceptedObjectsPointcloudPublisher<autoware_auto_perception_msgs::msg::DetectedObjects> {

    private:
        void objectsCallback(const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_objs_msg) override;
};

} // namespace detected_objects_pointcloud_publisher 

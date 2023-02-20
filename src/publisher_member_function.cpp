// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher")
  {
    // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud", 10);
    
    // subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //   "input/pointcloud", rclcpp::SensorDataQoS{}.keep_last(1),
    //    std::bind(&MinimalPublisher::pointCloudCallback, this, std::placeholders::_1));


    subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input/pointcloud", rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&MinimalPublisher::pointCloudCallback, this, std::placeholders::_1));

    // subscription_ = this->create_subscription<std_msgs::msg::String>(
    //   "topic", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg)
  {
    // auto message = std_msgs::msg::String();
    auto pointcloud_msg = sensor_msgs::msg::PointCloud2();
    pointcloud_msg.header = input_pointcloud_msg->header;
    pointcloud_msg.header.frame_id = "map";
    // message.data = "Hello, world! " + std::to_string(count_++);
    // RCLCPP_INFO(this->get_logger(), "Publishing pointcloud");
    RCLCPP_INFO(this->get_logger(), "I heard: '%s' at time '%s'", input_pointcloud_msg->header.frame_id.c_str(), rclcpp::Time);
    publisher_->publish(pointcloud_msg);
  }
  
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

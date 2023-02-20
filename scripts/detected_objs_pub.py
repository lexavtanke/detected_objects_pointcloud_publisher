import random

import rclpy
import autoware_perception_msgs

from rclpy.node import Node 
from autoware_auto_perception_msgs.msg import DetectedObjects
from autoware_auto_perception_msgs.msg import DetectedObject
from autoware_auto_perception_msgs.msg import ObjectClassification
from geometry_msgs.msg import Point32


class ObjsPublisher(Node):
    objects: list[DetectedObject]

    def __init__(self):
        super().__init__('detected_objects_pub')
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.pub = self.create_publisher(DetectedObjects, '/perception/object_recognition/objects', 10)

        self.objects = []

        count_obj_x = 5
        count_obj_y = 5 
        for i in range(count_obj_x):
            for j in range(count_obj_y):
                detected_object = DetectedObject()

                object_classification = ObjectClassification()
                object_classification.label = 1
                object_classification.probability = 1.0
                detected_object.classification.append(object_classification) # weird but only this way works

                detected_object.shape.type = 2 

                height = 4.0
                width = random.uniform(2, 5)

                detected_object.shape.dimensions.x = width
                detected_object.shape.dimensions.y = width
                detected_object.shape.dimensions.z = height

                detected_object.shape.footprint.points = []
                detected_object.shape.footprint.points.append(Point32(x=-width / 2.0, y=-width / 2.0))
                detected_object.shape.footprint.points.append(Point32(x=-width / 2.0, y=width / 2.0))
                detected_object.shape.footprint.points.append(Point32(x=width / 2.0, y=width / 2.0))
                detected_object.shape.footprint.points.append(Point32(x=width / 2.0, y=-width / 2.0))

                x = float(i) * 10  - count_obj_x / 2.0
                y = float(j) * 5  - count_obj_y / 2.0
                detected_object.kinematics.pose_with_covariance.pose.position.x = x
                detected_object.kinematics.pose_with_covariance.pose.position.y = y
                detected_object.kinematics.pose_with_covariance.pose.position.z = height / 2.0

                self.objects.append(detected_object)
    
    def timer_callback(self):
        self.get_logger().info('---timer_callback started---')
        detected_objects = DetectedObjects()
        detected_objects.header.frame_id = 'map'
        detected_objects.header.stamp = self.get_clock().now().to_msg()
        detected_objects.objects = self.objects

        self.pub.publish(detected_objects)


def main(args=None):
    rclpy.init(args=args)

    node = ObjsPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
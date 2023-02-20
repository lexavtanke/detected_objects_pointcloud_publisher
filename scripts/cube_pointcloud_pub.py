import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np


class CubePointCloudGenerator(Node):
    def __init__(self):
        super().__init__('cube_pointcloud_generator')
        self.publisher = self.create_publisher(PointCloud2, '/perception/obstacle_segmentation/pointcloud', 10)
        self.timer = self.create_timer(1.0, self.generate_and_publish_pointcloud)

    
    
    def generate_and_publish_pointcloud(self):
        vertices = self.generate()

        # Construct the point cloud message
        header = Header()
        header.frame_id = 'map'
        header.stamp = self.get_clock().now().to_msg()

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            # PointField(name="rgb",offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        point_cloud_msg = PointCloud2()
        point_cloud_msg.header = header
        point_cloud_msg.height = 1
        point_cloud_msg.width = vertices.shape[0]
        point_cloud_msg.fields = fields
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.point_step = 12
        point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width
        point_cloud_msg.is_dense = True

        point_cloud_msg.data = np.array(vertices, dtype=np.float32).tostring()

        # Publish the point cloud message
        self.publisher.publish(point_cloud_msg)

    def generate(self):
        x_min, x_max = -20, 40
        y_min, y_max = -20, 50
        z_min, z_max = -5, 5

        x = np.linspace(x_min,x_max,40)
        y = np.linspace(y_min,y_max,40)
        z = np.linspace(z_min,z_max,20)
        # using list comprehension 
        # to compute all possible permutations
        res = [[i, j, k] for i in x 
                         for j in y
                         for k in z]
        res = np.array(res)
        res = res.reshape(-1,3)
        return res



def main(args=None):
    rclpy.init(args=args)

    node = CubePointCloudGenerator()
    # node.generate()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

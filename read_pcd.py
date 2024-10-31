#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import icp_algo as icp
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('point_cloud_subscriber')
        
        # Create a subscriber to the PointCloud2 topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/agvr/point_cloud_raw',  
            self.point_cloud_callback,
            10  # QoS history depth
        )
        self.subscription  

        self.publisher = self.create_publisher(PointCloud2, '/transformed_template', 10)

    def point_cloud_callback(self, msg):
        self.get_logger().info('Received PointCloud2 data')
        self.get_logger().info(f'Width: {msg.width}, Height: {msg.height}, Fields: {len(msg.fields)}, Data Length: {len(msg.data)}')

        data_raw = np.array(msg.data)
        data = data_raw.reshape(-1, 5)

        template_file = np.load("./datasets/fiducial_plane.npz")
        template = template_file['point_cloud']

        print(template.shape)

        data_xyz = data[:, :3]
        template_xyz = template[:, :3]

        transformation, transformed_xyz = icp.icp(template_xyz, data_xyz)

        print(data_xyz.shape)
        print(transformed_xyz.shape)
        transformed_pcd2 = create_pointcloud2_from_xyz(np_points = transformed_xyz)
        self.publisher.publish(transformed_pcd2)
        self.get_logger().info('Published PointCloud2 message to /transformed_template')


        # print(data.shape)
        # print
        # for i in range(0, msg.)
        # You can also process the point cloud data (e.g., extract points)

def create_pointcloud2_from_xyz(np_points, frame_id="base_link", timestamp=None):

    print(np_points.shape)
    # Ensure the numpy array has the right shape
    assert np_points.shape[1] == 3, "Input numpy array must be Nx3 for xyz points"

    # Define the header with the frame id and timestamp
    header = Header()
    header.frame_id = frame_id
    if timestamp is not None:
        header.stamp = timestamp
    else:
        header.stamp = rclpy.time.Time().to_msg()

    # Create PointCloud2 from numpy array
    point_cloud_msg = point_cloud2.create_cloud_xyz32(header, np_points.tolist())
    # point_cloud_msg.
    
    return point_cloud_msg 

def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber()

    rclpy.spin(point_cloud_subscriber)

    point_cloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

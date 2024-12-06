#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import algorithms.icp_algo as icp
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 
import algorithms.icp_registration as icp_reg
import algorithms.feature_matches as feature_match
import struct

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

        scene = pointcloud2_to_xyz_array(msg)

        # Load in tank and template files
        cap_template = np.load("./datasets/cap.npy")
        tank_template= np.load("./datasets/gas_tank26.npy")

        # Reformat the scene to only grab xyz data
        scene_xyz = scene[:, :3]

        # Run the feature matching algorithm for good inital guess
        transformation, transformed_tank_template = feature_match.feature_matching(tank_template, scene_xyz)

        # Run ICP for fine tune of the template with the ransac as good inital guess
        transformation, transformed_cap_template =icp_reg.icp(cap_template, scene_xyz, transformation)

        transformed_pcd2 = create_pointcloud2_from_xyz(np_points = transformed_cap_template)
        self.publisher.publish(transformed_pcd2)
        self.get_logger().info('Published PointCloud2 message to /transformed_template')

def create_pointcloud2_from_xyz(np_points, frame_id="base_link", timestamp=None):

    print(np_points.shape)
    # assert np_points.shape[1] == 3, "Input numpy array must be Nx3 for xyz points"
    header = Header()
    header.frame_id = frame_id
    if timestamp is not None:
        header.stamp = timestamp
    else:
        header.stamp = rclpy.time.Time().to_msg()

    # Create PointCloud2 from numpy array
    point_cloud_msg = point_cloud2.create_cloud_xyz32(header, np_points.tolist())
    return point_cloud_msg 

def pointcloud2_to_xyz_array(pointcloud):
    # Initialize an empty list to hold the (x, y, z) values
    points = []

    # Define the format for a single point (assuming 'x', 'y', 'z' are float32)
    field_names = [field.name for field in pointcloud.fields]
    fmt = 'fff'  # For x, y, z as float32
    point_step = pointcloud.point_step

    # Iterate over each point in the data
    for i in range(0, len(pointcloud.data), point_step):
        # Unpack the binary data of each point
        x, y, z = struct.unpack_from(fmt, pointcloud.data, offset=i)
        
        # Check for valid points
        if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
            points.append([x, y, z])

    # Convert the list to a numpy array
    return np.array(points, dtype=np.float32)

def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber()

    rclpy.spin(point_cloud_subscriber)

    point_cloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

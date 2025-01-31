import rclpy
from rclpy.node import Node
import math
import numpy as np
import struct
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from custom_msg_interfaces.msg import DetectionResult
import tf2_ros
from geometry_msgs.msg import TransformStamped


class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscriptions
        self.create_subscription(
            DetectionResult, 
            '/detection_result_stamped', 
            self.detection_callback, 
            10
        )
        self.create_subscription(
            PointCloud2, 
            '/camera/image_raw/points', 
            self.pointcloud_callback, 
            10
        )
        
        # Store detection data here
        self.last_detection = None

        self.get_logger().info('pose estimator started.')

        self.obj_x = 0
        self.obj_y = 0

    def detection_callback(self, msg: DetectionResult):
        """
        Receives detection data from a custom message. 
        Suppose `msg.data` might look like: [some_id, center_x_px, center_y_px, ...]
        Adjust indices to match your actual message.
        """
        # Ensure data array has enough elements for your indexing
        if len(msg.data) < 3:
            self.get_logger().warn("Invalid detection data received.")
            return
        
        self.last_detection = msg.data
        self.obj_x = self.last_detection[2]
        self.obj_y = self.last_detection[3]
        self.get_logger().info(f"Detection data received: obj_x={self.obj_x:.3f}, obj_y={self.obj_y:.3f}")

    def pointcloud_callback(self, msg):
        width  = msg.width
        height = msg.height
        # Example pixel location (center_x, center_y)
        center_x = int(self.obj_x * width)
        center_y = int(self.obj_y * height)

        point = self.get_3d_point_from_2d(msg, center_x, center_y)
        if point:
            self.get_logger().info(f'3D Point at ({center_x}, {center_y}): x={point[0]:.3f}, y={point[1]:.3f}, z={point[2]:.3f}')
        else:
            self.get_logger().warn(f'Invalid point at ({center_x}, {center_y})')
        self.publish_transform(point)

    def get_3d_point_from_2d(self, cloud, x, y):
        """Extracts (x, y, z) coordinates from a given pixel (x, y) in PointCloud2."""
        if x < 0 or x >= cloud.width or y < 0 or y >= cloud.height:
            return None  # Out of bounds

        point_step = cloud.point_step  # 24 bytes per point
        row_step = cloud.row_step  # Bytes per row
        index = (y * row_step) + (x * point_step)

        fmt = 'fff'  # Extract x, y, z (FLOAT32 each)
        try:
            x, y, z = struct.unpack_from(fmt, cloud.data, offset=index)
            return (x, y, z)
        except struct.error:
            return None


        
    def publish_transform(self, point: tuple):
        """
        Publishes transforms from 'camera_rgb_frame' to 'object_frame'
        and from 'object_frame' to 'grasping_frame'.
        """
        x, y, z = point
        timestamp = self.get_clock().now().to_msg()

        # Object frame relative to camera_rgb_frame
        transform = TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = 'camera_rgb_frame'
        transform.child_frame_id = 'object_frame'
        
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        
        # Fixed orientation (Identity or estimated)
        transform.transform.rotation.x = -0.00595919
        transform.transform.rotation.y = -0.19858
        transform.transform.rotation.z = +0.0293976
        transform.transform.rotation.w = 0.979626
        
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info(
            f"Published transform: object_frame at ({x:.3f}, {y:.3f}, {z:.3f})"
        )

        # Grasping frame relative to object_frame
        transform_grasping = TransformStamped()
        transform_grasping.header.stamp = timestamp  # Use the same timestamp
        transform_grasping.header.frame_id = 'object_frame'
        transform_grasping.child_frame_id = 'grasping_frame'

        transform_grasping.transform.translation.x = 0.015
        transform_grasping.transform.translation.y = 0.0
        transform_grasping.transform.translation.z = 0.05 # Offset from object_frame

        # Identity orientation
        transform_grasping.transform.rotation.x = 1.0
        transform_grasping.transform.rotation.y = 0.0
        transform_grasping.transform.rotation.z = 0.0
        transform_grasping.transform.rotation.w = 0.0
        
        self.tf_broadcaster.sendTransform(transform_grasping)
        self.get_logger().info(
            f"Published transform: grasping_frame at (0.000, 0.000, 0.040) relative to object_frame"
        )



def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


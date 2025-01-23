import rclpy
from rclpy.node import Node
import struct
import sensor_msgs.msg as sensor_msgs

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        self.subscription = self.create_subscription(
            sensor_msgs.PointCloud2,
            '/camera/image_raw/points',
            self.pointcloud_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def pointcloud_callback(self, msg):
        # Example pixel location (center_x, center_y)
        center_x = 364  # Example: Center of 640x360 image
        center_y = 179  # Example: Center of 640x360 image

        point = self.get_3d_point_from_2d(msg, center_x, center_y)
        if point:
            self.get_logger().info(f'3D Point at ({center_x}, {center_y}): x={point[0]:.3f}, y={point[1]:.3f}, z={point[2]:.3f}')
        else:
            self.get_logger().warn(f'Invalid point at ({center_x}, {center_y})')

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

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

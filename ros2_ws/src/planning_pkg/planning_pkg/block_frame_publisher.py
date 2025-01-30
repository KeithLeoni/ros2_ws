import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class BlockFramePublisher(Node):
    def __init__(self):
        super().__init__('block_frame_publisher')

        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish transforms every 10ms (100Hz)
        self.timer = self.create_timer(0.01, self.publish_transforms)

        # Define block positions and orientations relative to "base"
        self.block_positions = {
            "block1_position": (0.45, 0.1, 0.9),
            "block2_position": (0.45, -0.1, 0.9),
            "block3_position": (0.45, -0.3, 0.9),
            "block1_grasping": (0.45, 0.1, 0.70),
            "block2_grasping": (0.45, -0.1, 0.70),
            "block3_grasping": (0.45, -0.3, 0.70),
            "block1_safe_plane": (0.45, 0.1, 0.5),
            "block2_safe_plane": (0.45, -0.1, 0.5),
            "block3_safe_plane": (0.45, -0.3, 0.5),
        }

        # Define block orientations (Quaternion: x, y, z, w)
        self.block_orientations = {
            "block1_position": (1.0, 0.0, 0.0, 0.0),
            "block2_position": (1.0, 0.0, 0.0, 0.0),
            "block3_position": (1.0, 0.0, 0.0, 0.0),
            "block1_grasping": (0.0, 0.0, 0.0, 1.0),
            "block2_grasping": (0.0, 0.0, 0.0, 1.0),
            "block3_grasping": (0.0, 0.0, 0.0, 1.0),
            "block1_safe_plane": (0.0, 0.0, 0.0, 1.0),
            "block2_safe_plane": (0.0, 0.0, 0.0, 1.0),
            "block3_safe_plane": (0.0, 0.0, 0.0, 1.0),
        }

    def publish_transforms(self):
        timestamp = self.get_clock().now().to_msg()

        for block_name, position in self.block_positions.items():
            orientation = self.block_orientations[block_name]

            # Create transform message
            transform = TransformStamped()
            transform.header.stamp = timestamp
            transform.header.frame_id = "base"  # Parent frame
            transform.child_frame_id = block_name  # Block frame

            # Set translation (position)
            transform.transform.translation.x = position[0]
            transform.transform.translation.y = position[1]
            transform.transform.translation.z = position[2]

            # Set rotation (orientation)
            transform.transform.rotation.x = orientation[0]
            transform.transform.rotation.y = orientation[1]
            transform.transform.rotation.z = orientation[2]
            transform.transform.rotation.w = orientation[3]

            # Publish transform
            self.tf_broadcaster.sendTransform(transform)

        #self.get_logger().info("Published 3 block transforms relative to 'base'")

def main(args=None):
    rclpy.init(args=args)
    node = BlockFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

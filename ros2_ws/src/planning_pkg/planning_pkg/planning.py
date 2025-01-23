import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class SafePlaneFramePublisher(Node):
    def __init__(self):
        super().__init__('safe_plane_frame_publisher')

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to publish transforms
        self.timer = self.create_timer(0.01, self.publish_transforms)

        self.plane_height = 1.4

    def publish_transforms(self):
        try:
            # Lookup transform from 'desk' (or world frame) to 'flange'
            flange_transform: TransformStamped = self.tf_buffer.lookup_transform(
                'desk', 'flange', rclpy.time.Time()
            )


            # Extract absolute X and Y positions
            x_abs = flange_transform.transform.translation.x
            y_abs = flange_transform.transform.translation.y

            self.get_logger().info(f'Flange Absolute Position -> X: {x_abs}, Y: {y_abs}')

        except Exception as e:
            self.get_logger().error(f'Failed to get transform: {e}')
            return  # Exit if the transform lookup fails

        timestamp = self.get_clock().now().to_msg()

        # Object frame relative to camera_rgb_frame
        transform_last_safe_plane = TransformStamped()
        transform_last_safe_plane.header.stamp = timestamp
        transform_last_safe_plane.header.frame_id = 'grasping_frame'
        transform_last_safe_plane.child_frame_id = 'last_safe_plane'

        transform_last_safe_plane.transform.translation.x = 0.0
        transform_last_safe_plane.transform.translation.y = 0.0
        transform_last_safe_plane.transform.translation.z = 0.5  # Fixed height

        # Fixed orientation (Identity or estimated)
        transform_last_safe_plane.transform.rotation.x = 0.0
        transform_last_safe_plane.transform.rotation.y = 0.0
        transform_last_safe_plane.transform.rotation.z = 0.0
        transform_last_safe_plane.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(transform_last_safe_plane)
        '''
        # Grasping frame relative to last_safe_plane
        transform_grasping = TransformStamped()
        transform_grasping.header.stamp = timestamp  # Use the same timestamp
        transform_grasping.header.frame_id = 'desk'
        transform_grasping.child_frame_id = 'start_safe_plane'

        # Correct usage of flange_transform
        transform_grasping.transform.translation.x = x_abs 
        transform_grasping.transform.translation.y = y_abs
        transform_grasping.transform.translation.z = 1.5  # Offset from object_frame

        # Identity orientation
        transform_grasping.transform.rotation.x = 0.0
        transform_grasping.transform.rotation.y = 0.0
        transform_grasping.transform.rotation.z = 0.0
        transform_grasping.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform_grasping)
        '''
        



def main(args=None):
    rclpy.init(args=args)
    node = SafePlaneFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

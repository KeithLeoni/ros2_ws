#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Header
from custom_msg_interfaces.msg import DetectionResult

class DetectionResultRepublisher(Node):
    def __init__(self):
        super().__init__('detection_result_republisher')
        
        # Subscriber to the original detection result
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/detection_result',
            self.detection_callback,
            10)
        
        # Publisher for the stamped detection result
        self.publisher = self.create_publisher(
            DetectionResult,
            '/detection_result_stamped',
            2)
        
        self.get_logger().info('Detection Result Republisher Node started.')

    def detection_callback(self, msg):
        # Log the received message
        #self.get_logger().info(f'Received detection result with {len(msg.data)} elements.')
        
        # Create a DetectionResult message
        stamped_msg = DetectionResult()
        stamped_msg.header = Header()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()  # Add timestamp
        stamped_msg.header.frame_id = 'camera_rgb_frame'  # Set frame ID (optional)
        stamped_msg.data = msg.data  # Copy data from the received message
        
        # Publish the stamped message
        self.publisher.publish(stamped_msg)
        #self.get_logger().info(f'Published stamped detection result.')

def main(args=None):
    rclpy.init(args=args)
    node = DetectionResultRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

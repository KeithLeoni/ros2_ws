#!/home/ubuntu/ros2_env/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class YoloOverlayNode(Node):
    def __init__(self):
        super().__init__('yolo_overlay_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Variables to store image and detections
        self.current_image = None
        self.current_detections = None
        
        # Subscribe to the camera topic
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw/image',
            self.image_callback,
            10)
        
        # Subscribe to the detection results
        self.detection_subscription = self.create_subscription(
            Float32MultiArray,
            '/detection_result',
            self.detection_callback,
            10)
        
        # Publisher for overlayed video stream
        self.publisher = self.create_publisher(Image, '/detection_overlay', 10)
        
        self.get_logger().info('YOLO Overlay Node started.')
    
    def image_callback(self, msg):
        """Callback to handle incoming images."""
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.publish_overlay()
    
    def detection_callback(self, msg):
        """Callback to handle incoming detections."""
        # Reshape to [class, confidence, center_x, center_y, width, height]
        self.current_detections = np.array(msg.data).reshape(-1, 6)
    
    def publish_overlay(self):
        """Draw bounding boxes and publish the overlayed image."""
        if self.current_image is None or self.current_detections is None:
            return
        
        overlay_image = self.current_image.copy()
        height, width, _ = overlay_image.shape  # Auto-detect frame dimensions
        #self.get_logger().info(f"Image Dimensions: Width={height}, Height={width}, Channels={_}")

        
        # Draw each detection as a bounding box
        for detection in self.current_detections:
            class_id, confidence, center_x, center_y, w, h = detection
            
            # Convert normalized values to absolute pixel values
            center_x = int(center_x * width)
            center_y = int(center_y * height)
            w = int(w * width)
            h = int(h * height)
            
            # Calculate top-left corner from center coordinates
            x_min = center_x - w // 2
            y_min = center_y - h // 2
            
            # Draw rectangle and label
            cv2.rectangle(overlay_image, (x_min, y_min), (x_min + w, y_min + h), (0, 255, 0), 2)
            label = f'Class: {int(class_id)} Confidence: {confidence:.2f}'
            cv2.putText(overlay_image, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Convert back to ROS Image message and publish
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay_image, encoding='bgr8')
        self.publisher.publish(overlay_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloOverlayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


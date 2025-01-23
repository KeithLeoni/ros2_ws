#!/home/ubuntu/ros2_env/bin/python3
import sys
print("Using Python interpreter:", sys.executable)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # Load YOLOv8 model
        self.model = YOLO('runs/detect/yolo11s_big_dataset3/weights/best.pt')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw/image',
            self.image_callback,
            10)
        
        # Publisher for detection results
        self.publisher = self.create_publisher(Float32MultiArray, '/detection_result', 10)
        
        self.get_logger().info('YOLO Detector Node started.')
    
    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        resized_image = cv2.resize(cv_image, (640, 640))

        # Run YOLO inference and extract detection data
        results = self.model(resized_image)
        boxes = results[0].boxes.xywhn.cpu().numpy()  # Normalized [center_x, center_y, width, height]
        confidences = results[0].boxes.conf.cpu().numpy()  # Confidence scores
        class_ids = results[0].boxes.cls.cpu().numpy().astype(int)  # Class IDs

        if len(confidences) == 0:
            self.get_logger().info('No detections found.')
            return

        # Uncomment one of the approaches below as needed

        # Approach 1: Detect all classes above a threshold
        '''
        flat_result = [
            val for i in range(len(boxes)) if confidences[i] > 0.6
            for val in [class_ids[i], confidences[i], *boxes[i]]
        ]
        
        if flat_result:  # Publish only if there are detections
            msg = Float32MultiArray(data=flat_result)
            self.publisher.publish(msg)
            self.get_logger().info(f'Published {len(flat_result) // 6} detections.')
        else:
            self.get_logger().info('No detections with confidence > 0.6.')
        '''

        # Approach 2: Find the best detection (highest confidence)
        best_index = np.argmax(confidences)
        best_result = [
            class_ids[best_index],          # Class ID
            confidences[best_index],        # Confidence
            *boxes[best_index]              # [Center X, Center Y, Width, Height] Normalized
        ]
        
        msg = Float32MultiArray(data=best_result)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published best detection: Class {best_result[0]}, Confidence {best_result[1]:.2f}')



def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


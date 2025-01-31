#!/home/ubuntu/ros2_env/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
import struct
import os

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from custom_msg_interfaces.msg import DetectionResult
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_transformations


class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')
        
        # ----------------------
        # Declare a debug_mode parameter
        # ----------------------
        self.declare_parameter('debug_mode', True)
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        
        # If debug_mode is True, set this node's logger to DEBUG level
        if self.debug_mode:
            # This sets the node's log level (ROS2 Foxy+ supports this API)
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # --- Load STL / mesh and convert to point cloud ---
        home = os.path.expanduser("~")  # e.g. "/home/ubuntu"
        stl_path = os.path.join(
            home,
            "ros2_ws",
            "src",
            "pose_estimator_pkg",
            "pose_estimator_pkg",
            "stl",
            "X1-Y1-Z2.stl"
        )
        mesh = o3d.io.read_triangle_mesh(stl_path)
        self.object_pcd = mesh.sample_points_uniformly(number_of_points=2000)

        # Build TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Store detection data
        self.obj_x = 0.0
        self.obj_y = 0.0

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

        self.get_logger().info('ICP pose estimator started.')

    def detection_callback(self, msg: DetectionResult):
        """Receives the 2D detection coordinates (normalized or pixel-based)."""
        if len(msg.data) < 4:
            self.get_logger().warn("Invalid detection data.")
            if self.debug_mode:
                self.get_logger().debug(f"Received detection data: {msg.data}")
            return

        # Adjust indices if needed.
        self.obj_x = msg.data[2]
        self.obj_y = msg.data[3]
        self.get_logger().info(f"Detection data: obj_x={self.obj_x:.3f}, obj_y={self.obj_y:.3f}")

    def pointcloud_callback(self, msg: PointCloud2):
        """Converts ROS PointCloud2 into Open3D and runs ICP using the detection info."""
        scene_pcd = self.ros_to_o3d(msg)
        num_scene_points = np.asarray(scene_pcd.points).shape[0]
        self.get_logger().info(f"Number of scene points: {num_scene_points}")

        # If the scene is empty or extremely sparse, skip
        if num_scene_points < 10:
            self.get_logger().warn("Scene point cloud is too small. Skipping ICP.")
            if self.debug_mode:
                self.get_logger().debug("Point cloud likely has too few points for reliable ICP.")
            return

        width, height = msg.width, msg.height
        center_x = int(self.obj_x * width)
        center_y = int(self.obj_y * height)
        
        if self.debug_mode:
            self.get_logger().debug(f"Computed pixel coords from detection: ({center_x}, {center_y})")

        approximate_3d_point = self.get_3d_point_from_2d(msg, center_x, center_y)
        if approximate_3d_point is None:
            self.get_logger().warn("No valid 3D point from detection. Skipping ICP.")
            if self.debug_mode:
                self.get_logger().debug(
                    f"No 3D point at pixel ({center_x},{center_y}). Possibly invalid depth or out of range."
                )
            return

        # Build a simple initial guess for ICP
        init_guess = np.eye(4)
        init_guess[0:3, 3] = approximate_3d_point  # translation part

        # Run ICP
        icp_T = self.run_icp(self.object_pcd, scene_pcd, init_guess)

        # Broadcast transform
        self.broadcast_transform(icp_T)

    def get_3d_point_from_2d(self, cloud: PointCloud2, x: int, y: int):
        """Extracts a single (X, Y, Z) from the point cloud at pixel (x, y)."""
        if x < 0 or x >= cloud.width or y < 0 or y >= cloud.height:
            if self.debug_mode:
                self.get_logger().debug(f"Pixel ({x},{y}) is out of bounds for the cloud.")
            return None

        point_step = cloud.point_step
        row_step = cloud.row_step
        index = (y * row_step) + (x * point_step)
        fmt = 'fff'
        try:
            X, Y, Z = struct.unpack_from(fmt, cloud.data, offset=index)
            return np.array([X, Y, Z], dtype=np.float32)
        except struct.error:
            if self.debug_mode:
                self.get_logger().debug(f"Failed to unpack from cloud data at index {index}.")
            return None

    def ros_to_o3d(self, ros_cloud_msg: PointCloud2):
        """Converts a ROS PointCloud2 message into an Open3D PointCloud."""
        points_list = []
        for p in point_cloud2.read_points(ros_cloud_msg, skip_nans=True):
            x, y, z = p[0], p[1], p[2]
            points_list.append([x, y, z])

        np_points = np.asarray(points_list, dtype=np.float32)
        cloud_o3d = o3d.geometry.PointCloud()
        cloud_o3d.points = o3d.utility.Vector3dVector(np_points)
        return cloud_o3d

    def run_icp(self, object_pcd: o3d.geometry.PointCloud,
                scene_pcd: o3d.geometry.PointCloud,
                initial_transform: np.ndarray) -> np.ndarray:
        """
        Runs ICP using a voxel-downsampled object and scene.
        Dynamically choose a voxel size based on the scene bounding box
        to avoid "voxel_size is too small" errors.
        """
        # Estimate a suitable voxel size from the scene bounding box
        scene_bbox = scene_pcd.get_axis_aligned_bounding_box()
        scene_diag = np.linalg.norm(scene_bbox.get_extent())

        # As a rough rule, use ~1% of the bounding box diagonal
        voxel_size = max(scene_diag * 0.01, 0.001)  # clamp to >= 0.001

        # Downsample
        object_down = object_pcd.voxel_down_sample(voxel_size=voxel_size)
        scene_down = scene_pcd.voxel_down_sample(voxel_size=voxel_size)

        # Estimate normals for ICP
        object_down.estimate_normals()
        scene_down.estimate_normals()

        # Let's set the ICP matching threshold to something related to the voxel size
        threshold = voxel_size * 3.0  # 3x voxel size as a ballpark

        self.get_logger().info(
            f"Running ICP with voxel_size={voxel_size:.4f}, threshold={threshold:.4f}"
        )

        result_icp = o3d.pipelines.registration.registration_icp(
            object_down,
            scene_down,
            threshold,
            initial_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )

        if self.debug_mode:
            self.get_logger().debug(f"ICP fitness: {result_icp.fitness}, inlier_rmse: {result_icp.inlier_rmse}")

        return result_icp.transformation

    def broadcast_transform(self, icp_T: np.ndarray):
        """Publishes the resulting 4x4 transform as a TF."""

        # --- Check for NaN or invalid shape ---
        if icp_T.shape != (4, 4):
            self.get_logger().error(
                f"ICP transform has invalid shape: {icp_T.shape}. Expected (4,4). Skipping broadcast."
            )
            return
        if np.isnan(icp_T).any():
            self.get_logger().error("ICP transform contains NaN values. Skipping broadcast.")
            return

        trans = TransformStamped()
        trans.header.stamp = self.get_clock().now().to_msg()
        trans.header.frame_id = "camera_rgb_frame"
        trans.child_frame_id = "object_frame_icp"
        
        # Translation (from ICP)
        trans.transform.translation.x = float(icp_T[0, 3])
        trans.transform.translation.y = float(icp_T[1, 3])
        trans.transform.translation.z = float(icp_T[2, 3])

        # Rotation (from ICP)
        quat = tf_transformations.quaternion_from_matrix(icp_T)
        trans.transform.rotation.x = float(quat[0])
        trans.transform.rotation.y = float(quat[1])
        trans.transform.rotation.z = float(quat[2])
        trans.transform.rotation.w = float(quat[3])

        # Publish the ICP frame transform
        self.tf_broadcaster.sendTransform(trans)
        self.get_logger().info("Published ICP transform (object_frame_icp).")

        # ----------------------------------------------------------
        # Now, create a second TF for the "grasping_frame" with an
        # offset above object_frame_icp along the Z-axis.
        # ----------------------------------------------------------
        trans2 = TransformStamped()
        trans2.header.stamp = trans.header.stamp
        trans2.header.frame_id = "object_frame_icp"
        trans2.child_frame_id = "grasping_frame"

        # This translation is relative to the parent (object_frame_icp)
        trans2.transform.translation.x = 0.0
        trans2.transform.translation.y = 0.0
        trans2.transform.translation.z = 0.15  # e.g., 15 cm above

        # Keep same orientation as the parent
        trans2.transform.rotation.x = 1.0
        trans2.transform.rotation.y = 0.0
        trans2.transform.rotation.z = 0.0
        trans2.transform.rotation.w = 0.0

        self.tf_broadcaster.sendTransform(trans2)
        self.get_logger().info("Published grasping_frame transform, offset above object_frame_icp.")


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

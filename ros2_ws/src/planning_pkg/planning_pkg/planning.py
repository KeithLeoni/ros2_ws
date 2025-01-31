import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Pose, PoseArray
from std_msgs.msg import Header, String
import tf2_geometry_msgs  # for do_transform_pose, etc.
import subprocess

from custom_msg_interfaces.srv import MoveAB
from std_msgs.msg import Float32MultiArray


class SafePlaneFramePublisher(Node):
    def __init__(self):
        super().__init__('safe_plane_frame_publisher')

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers and Subscribers
        self.planned_poses_pub = self.create_publisher(PoseArray, 'planned_poses', 10)  # /planned_poses
        self.subscription = self.create_subscription(Float32MultiArray, '/detection_result', self.detection_callback, 10)
        self.create_subscription(String, 'trajectory_done', self.trajectory_done_callback, 10)

        # We broadcast transforms only if needed. (Not shown in this simplified flow)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Parameters / States
        self.plane_height = 0.5
        self.detected_class = None
        self.planned_poses = []
        self.current_pose_index = 0

        # Timer just checks if we have a detection yet (could be removed if you want a purely event-driven flow)
        self.timer = self.create_timer(1.0, self.check_and_plan)

        self.get_logger().info('SafePlaneFramePublisher node started.')

    def detection_callback(self, msg):
        """Called when a detection result arrives."""
        data = msg.data
        if len(data) < 6:
            self.get_logger().error("Invalid detection result format.")
            return

        self.detected_class = data[0]
        self.get_logger().info(f"Detected class: {self.detected_class}")

        # Unsubscribe after first valid detection to avoid repeated triggers
        self.destroy_subscription(self.subscription)
        self.get_logger().info("Unsubscribed from /detection_result")

    def check_and_plan(self):
        """
        Periodically checks if we have a detection.
        If we do and haven't planned yet, compute and publish the sequence of poses.
        """
        if self.detected_class is None:
            self.get_logger().warn("Waiting for detection...")
            return

        # Once we have a detection, compute the plan.
        if not self.planned_poses:  # Only do this once
            self.compute_and_publish_plan()

    def compute_and_publish_plan(self):
        """Compute the sequence of poses and publish them. Then start moving."""
        try:
            # Pose0: Current Tool Pose
            pose0 = self.transform_stamped_to_pose(self.get_transform('base', 'tool0'))

            # Pose1: Same X/Y but at plane_height
            pose1 = Pose()
            pose1.position.x = pose0.position.x
            pose1.position.y = pose0.position.y
            pose1.position.z = self.plane_height
            pose1.orientation = pose0.orientation

            # Pose2 / Pose3 come from 'grasping_frame'
            tfs_base_grasp = self.get_transform('base', 'grasping_frame')
            pose2_raw = self.transform_stamped_to_pose(tfs_base_grasp)
            pose2 = Pose()
            pose2.position.x = pose2_raw.position.x
            pose2.position.y = pose2_raw.position.y
            pose2.position.z = self.plane_height
            pose2.orientation = pose2_raw.orientation


            
            #not icp
            pose3 = Pose()
            pose3.position.x = pose2_raw.position.x
            pose3.position.y = pose2_raw.position.y
            pose3.position.z = pose2_raw.position.z - 0.09
            pose3.orientation = pose2_raw.orientation
            '''

            #icp
            pose3 = Pose()
            pose3.position.x = pose2_raw.position.x
            pose3.position.y = pose2_raw.position.y
            pose3.position.z = pose2_raw.position.z
            pose3.orientation = pose2_raw.orientation
            '''

            # Decide which block transforms to use
            if self.detected_class == 5:
                safe_plane_frame = 'block2_safe_plane'
                grasping_frame = 'block2_grasping'
            elif self.detected_class == 7:
                safe_plane_frame = 'block3_safe_plane'
                grasping_frame = 'block3_grasping'
            else:
                safe_plane_frame = 'block1_safe_plane'
                grasping_frame = 'block1_grasping'

            pose4 = self.transform_stamped_to_pose(self.get_transform('base', safe_plane_frame))
            pose5 = self.transform_stamped_to_pose(self.get_transform('base', grasping_frame))

            # Plan: [pose0, pose1, pose2, pose3, pose2, pose4, pose5, pose4]
            self.planned_poses = [pose0, pose1, pose2, pose3, pose2, pose4, pose5, pose4]

            # Publish PoseArray for visualization
            planned_poses_msg = PoseArray()
            planned_poses_msg.header = Header()
            planned_poses_msg.header.stamp = self.get_clock().now().to_msg()
            planned_poses_msg.header.frame_id = 'base'
            planned_poses_msg.poses.extend(self.planned_poses)
            self.planned_poses_pub.publish(planned_poses_msg)

            self.get_logger().info("Planned poses published. Executing now...")
            self.current_pose_index = 0
            self.move_to_next_pose()  # Start executing

        except Exception as e:
            self.get_logger().error(f"Error computing plan: {e}")

    def move_to_next_pose(self):
        """
        Moves the robot from self.planned_poses[current_pose_index]
        to self.planned_poses[current_pose_index + 1], then waits
        for 'trajectory_done' to continue.
        """
        if self.current_pose_index >= len(self.planned_poses) - 1:
            self.get_logger().info("All poses executed. Sequence complete.")
            return

        pose_a = self.planned_poses[self.current_pose_index]
        pose_b = self.planned_poses[self.current_pose_index + 1]

        # Example: open/close gripper at specific transitions
        if self.current_pose_index == 2:
            self.open_gripper()
        elif self.current_pose_index == 3:
            self.close_gripper()
        elif self.current_pose_index == 6:
            self.open_gripper()

        self.get_logger().info(f"Moving from pose {self.current_pose_index} to {self.current_pose_index+1}")
        self.call_move_service(pose_a, pose_b)

    def trajectory_done_callback(self, msg):
        """Proceeds to next pose upon completion of the current trajectory."""
        if "completed successfully" in msg.data.lower():
            self.current_pose_index += 1
            self.move_to_next_pose()

    def call_move_service(self, pose_a: Pose, pose_b: Pose):
        """
        Calls a MoveAB service to execute the motion.
        We rely on 'trajectory_done' topic to know when it's actually finished.
        """
        client = self.create_client(MoveAB, 'move_a_to_b')
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("'move_a_to_b' service is not available.")
            return

        req = MoveAB.Request()
        req.pose_start = pose_a
        req.pose_end = pose_b

        future = client.call_async(req)
        # We don't strictly need a done_callback here since we rely on trajectory_done topic.
        # But we can keep it if we want service result info:
        future.add_done_callback(self.move_service_callback)

    def move_service_callback(self, future):
        """Logs the result of the MoveAB service call."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("MoveAB service call succeeded.")
            else:
                self.get_logger().info(f"target acquired! moving the arm ;)")
        except Exception as e:
            #self.get_logger().error(f"Service call failed: {e}")
            pass

    def open_gripper(self):
        self.get_logger().info("Open gripper")
        self.call_trigger_service("/open_gripper")

    def close_gripper(self):
        self.get_logger().info("Close gripper")
        self.call_trigger_service("/close_gripper")

    def call_trigger_service(self, service_name: str):
        """Example of calling a Trigger service via subprocess (could also use a proper service client)."""
        try:
            result = subprocess.run(
                ["ros2", "service", "call", service_name, "std_srvs/srv/Trigger", "{}"],
                capture_output=True, text=True
            )
            if result.stdout:
                self.get_logger().info(f"Gripper response: {result.stdout.strip()}")
            if result.stderr:
                self.get_logger().warn(f"Gripper error: {result.stderr.strip()}")
        except Exception as e:
            self.get_logger().error(f"Exception calling gripper service: {e}")

    def transform_stamped_to_pose(self, tfs: TransformStamped) -> Pose:
        """
        Converts a TransformStamped into a geometry_msgs/Pose.
        """
        pose = Pose()
        pose.position.x = tfs.transform.translation.x
        pose.position.y = tfs.transform.translation.y
        pose.position.z = tfs.transform.translation.z
        pose.orientation = tfs.transform.rotation
        return pose

    def get_transform(self, parent: str, child: str) -> TransformStamped:
        """
        Looks up a transform (parent -> child) with a short timeout.
        """
        return self.tf_buffer.lookup_transform(
            parent, child, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.5)
        )


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

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Pose, PoseArray
from std_msgs.msg import Header, String
import tf2_geometry_msgs  # for do_transform_pose, etc.
import subprocess
from custom_msg_interfaces.srv import MoveAB


class SafePlaneFramePublisher(Node):
    def __init__(self):
        super().__init__('safe_plane_frame_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.planned_poses_pub = self.create_publisher(PoseArray, 'planned_poses', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.execute_sequence)
        self.plane_height = 0.5
        self.confirmed = False
        self.planned_poses = []
        self.current_pose_index = 0
        self.executing = False  # Flag to track if movement is in progress

        self.create_subscription(String, 'trajectory_done', self.trajectory_done_callback, 10)

        self.get_logger().info('SafePlaneFramePublisher node started.')

    def transform_stamped_to_pose(self, tfs: TransformStamped) -> Pose:
        pose = Pose()
        pose.position.x = tfs.transform.translation.x
        pose.position.y = tfs.transform.translation.y
        pose.position.z = tfs.transform.translation.z
        pose.orientation = tfs.transform.rotation
        return pose

    def get_transform(self, parent: str, child: str) -> TransformStamped:
        try:
            tfs = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            return tfs
        except Exception as e:
            self.get_logger().error(f'Could not lookup transform {parent} -> {child}: {e}')
            raise

    def execute_sequence(self):
        if self.confirmed or self.executing:
            return  # Stop publishing if confirmed or executing

        try:
            tfs_base_flange = self.get_transform('base', 'tool0')
            pose0 = self.transform_stamped_to_pose(tfs_base_flange)

            pose1 = Pose()
            pose1.position.x = pose0.position.x
            pose1.position.y = pose0.position.y
            pose1.position.z = self.plane_height
            pose1.orientation.x = pose0.orientation.x
            pose1.orientation.y = pose0.orientation.y
            pose1.orientation.z = pose0.orientation.z
            pose1.orientation.w = pose0.orientation.w

            tfs_base_grasp = self.get_transform('base', 'grasping_frame')
            pose2_raw = self.transform_stamped_to_pose(tfs_base_grasp)

            pose2 = Pose()
            pose2.position.x = pose2_raw.position.x
            pose2.position.y = pose2_raw.position.y
            pose2.position.z = self.plane_height
            pose2.orientation = pose2_raw.orientation

            pose3 = Pose()
            pose3.position.x = pose2_raw.position.x
            pose3.position.y = pose2_raw.position.y
            pose3.position.z = pose2_raw.position.z - 0.09
            pose3.orientation = pose2_raw.orientation

            tfs_base_safe_plane = self.get_transform('base', 'block1_safe_plane')
            pose4 = self.transform_stamped_to_pose(tfs_base_safe_plane)

            tfs_base_grasping = self.get_transform('base', 'block1_grapsing')
            pose5 = self.transform_stamped_to_pose(tfs_base_grasping)

            self.planned_poses = [pose0, pose1, pose2, pose3, pose2, pose4, pose5, pose4]


            planned_poses_msg = PoseArray()
            planned_poses_msg.header = Header()
            planned_poses_msg.header.stamp = self.get_clock().now().to_msg()
            planned_poses_msg.header.frame_id = 'base'
            planned_poses_msg.poses.extend(self.planned_poses)

            self.planned_poses_pub.publish(planned_poses_msg)
            self.get_logger().info('Published planned_poses with 5 poses.')

            # Start executing the movement after publishing planned poses
            self.executing = True
            self.execute_movement()

        except Exception as e:
            self.get_logger().error(f'Error in execute_sequence: {e}')

    def execute_movement(self):
        if len(self.planned_poses) < 2:
            self.get_logger().error("Not enough poses for movement.")
            return

        self.current_pose_index = 0
        self.move_robot_to_next_pose()

    def move_robot_to_next_pose(self):
        if self.current_pose_index >= len(self.planned_poses) - 1:  # Stop at last pose
            self.get_logger().info("Sequence completed. Stopping execution.")
            self.executing = False  # Stop execution
            return


        pose_a = self.planned_poses[self.current_pose_index]
        pose_b = self.planned_poses[self.current_pose_index + 1]  # Move to next pose

        # Print "Open Gripper" when transitioning from pose2 to pose3
        if self.current_pose_index == 2:  # Index of pose2
            self.get_logger().info("Open Gripper")
            self.call_open_gripper()

        if self.current_pose_index == 3:  # Index of pose3
            self.get_logger().info("Close Gripper")
            self.call_close_gripper()

        if self.current_pose_index == 4:  # Index of pose2
            self.get_logger().info("Open Gripper let it fall")
            self.call_open_gripper()

        if self.current_pose_index == 6:  # Index of pose2
            self.get_logger().info("Open Gripper")
            self.call_open_gripper()

        self.get_logger().info(f"Moving from pose {self.current_pose_index} to {self.current_pose_index + 1}")
        
        self.call_move_service(pose_a, pose_b)


    def call_move_service(self, pose_a, pose_b):
        self.get_logger().info("Calling MoveAB service")
        move_service = self.create_client(MoveAB, 'move_a_to_b')
        req = MoveAB.Request()
        req.pose_start = pose_a
        req.pose_end = pose_b

        if move_service.wait_for_service(timeout_sec=5.0):
            future = move_service.call_async(req)
            future.add_done_callback(self.move_service_callback)
        else:
            self.get_logger().error("MoveAB service unavailable.")

    def move_service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("MoveAB executed successfully.")
            else:
                self.get_logger().error(f"MoveAB failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def trajectory_done_callback(self, msg):
        self.get_logger().info(f"Received trajectory_done message: {msg.data}")
        if msg.data.lower() == "trajectory execution completed successfully!":
            self.current_pose_index += 1
            self.move_robot_to_next_pose()

    def call_close_gripper(self):
        try:
            result = subprocess.run(["ros2", "service", "call", "/close_gripper", "std_srvs/srv/Trigger", "{}"], capture_output=True, text=True)
            print("Service Call Output:\n", result.stdout)
            if result.stderr:
                print("Error:\n", result.stderr)
        except Exception as e:
            print("Exception occurred:", str(e))

    def call_open_gripper(self):
        try:
            result = subprocess.run(["ros2", "service", "call", "/open_gripper", "std_srvs/srv/Trigger", "{}"], capture_output=True, text=True)
            print("Service Call Output:\n", result.stdout)
            if result.stderr:
                print("Error:\n", result.stderr)
        except Exception as e:
            print("Exception occurred:", str(e))

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

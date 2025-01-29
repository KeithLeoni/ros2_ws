import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from custom_msg_interfaces.srv import ComputePath, ComputeTrajectory
from geometry_msgs.msg import PoseArray, Pose

class ComputePathAndTrajectoryClient(Node):
    def __init__(self):
        super().__init__('compute_path_and_trajectory_client')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Clients for ComputePath and ComputeTrajectory services
        self.compute_path_client = self.create_client(ComputePath, 'compute_path')
        self.compute_trajectory_client = self.create_client(ComputeTrajectory, 'compute_trajectory')

        # Action client for executing the trajectory
        self.action_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')

        # Trajectory publisher for RViz visualization
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/visualization_trajectory', qos_profile)

        # Wait for services
        while not self.compute_path_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for compute_path service...')
        while not self.compute_trajectory_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for compute_trajectory service...')
        while not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('Waiting for action server...')

        # Send initial request
        self.send_compute_path_request()

    def send_compute_path_request(self):
        """Sends request to compute_path service."""

        request = ComputePath.Request()
        
        #starting pose
        
        request.pose_start = Pose()
        request.pose_start.position.x = -0.143
        request.pose_start.position.y = -0.436
        request.pose_start.position.z = 0.206
        request.pose_start.orientation.x = 0.0
        request.pose_start.orientation.y = 1.0
        request.pose_start.orientation.z = 0.008
        request.pose_start.orientation.w = 0.015
	    
        
        request.pose_end = Pose()
        request.pose_end.position.x = -0.2
        request.pose_end.position.y = -0.5
        request.pose_end.position.z = 0.3
        request.pose_end.orientation.x = 0.5
        request.pose_end.orientation.y = 1.1
        request.pose_end.orientation.z = 0.03
        request.pose_end.orientation.w = 0.01

        request.num_interpolations = 4

        self.get_logger().info("Calling compute_path service...")
        self.future = self.compute_path_client.call_async(request)
        self.future.add_done_callback(self.handle_compute_path_response)

    def handle_compute_path_response(self, future):
        """Handles response from compute_path service and calls compute_trajectory service."""
        try:
            response = future.result()
            self.get_logger().info("Received Path from compute_path")
            pose_array = PoseArray()
            pose_array.poses = response.poses
            self.send_compute_trajectory_request(pose_array)
        except Exception as e:
            self.get_logger().error(f"compute_path service call failed: {e}")

    def send_compute_trajectory_request(self, pose_array):
        """Sends request to compute_trajectory service."""
        self.get_logger().info("Calling compute_trajectory service with PoseArray")
        request = ComputeTrajectory.Request()
        request.array = pose_array

        self.future = self.compute_trajectory_client.call_async(request)
        self.future.add_done_callback(self.handle_compute_trajectory_response)

    def handle_compute_trajectory_response(self, future):
        """Handles response from compute_trajectory service and sends trajectory if confirmed."""
        try:
            response = future.result()
            self.get_logger().info(f"Received response from compute_trajectory: {response.status_message}")

            # Check if trajectory is valid
            if not response.trajectory.joint_names or not response.trajectory.points:
                self.get_logger().error("Received an empty trajectory! Exiting...")
                return  # Return immediately if the trajectory is not valid

            # Publish trajectory to RViz
            self.get_logger().info(f"Publishing trajectory with {len(response.trajectory.points)} points to RViz")
            self.trajectory_pub.publish(response.trajectory)

            # Ask user for confirmation before sending to action server
            self.get_logger().info("Trajectory computed. Do you want to execute it? (yes/no)")

            user_input = input().strip().lower()
            if user_input in ["yes", "y"]:
                self.send_action_goal(response.trajectory)
            else:
                self.get_logger().info("Trajectory execution cancelled.")

        except Exception as e:
            self.get_logger().error(f"compute_trajectory service call failed: {e}")

    def send_action_goal(self, trajectory):
        """Sends the computed trajectory to the action server."""
        self.get_logger().info("Sending trajectory to action server...")

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        send_goal_options = rclpy.action.client.ClientGoalHandle
        self.future_goal = self.action_client.send_goal_async(goal_msg)
        self.future_goal.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handles response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory execution was rejected by the controller.")
            return

        self.get_logger().info("Trajectory execution accepted, waiting for result...")
        self.future_result = goal_handle.get_result_async()
        self.future_result.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handles the result of the action execution."""
        result = future.result()
        if result and result.result.error_code == 0:
            self.get_logger().info("Trajectory execution succeeded!")
        else:
            self.get_logger().error(f"Trajectory execution failed with error code: {result.result.error_code}")

        # Shutdown node after execution
        self.get_logger().info("finished and exiting node")

def main():
    rclpy.init()
    node = ComputePathAndTrajectoryClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


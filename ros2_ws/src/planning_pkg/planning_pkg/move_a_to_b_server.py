import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import threading  # Import threading for event-based waiting
from std_msgs.msg import Header, String
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseArray, Pose
from trajectory_msgs.msg import JointTrajectory

# Custom services
from custom_msg_interfaces.srv import MoveAB, ComputePath, ComputeTrajectory


class MoveAToBServer(Node):
    def __init__(self):
        super().__init__('move_a_to_b_server')

        # Create the service: 'move_a_to_b'
        self.srv = self.create_service(MoveAB, 'move_a_to_b', self.handle_move_a_to_b)

        # Clients for ComputePath and ComputeTrajectory services
        self.compute_path_client = self.create_client(ComputePath, 'compute_path')
        self.compute_trajectory_client = self.create_client(ComputeTrajectory, 'compute_trajectory')

        # Action client for executing joint trajectories
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info("MoveAToBServer initialized and ready.")
        
        # Publisher to notify when trajectory execution is done
        self.trajectory_done_publisher = self.create_publisher(String, 'trajectory_done', 10)

    def result_callback(self, future, response):
        """Handles the result of the action execution and publishes a message when done."""
        try:
            result = future.result()
            msg = String()

            if result and result.result.error_code == 0:
                self.get_logger().info("Trajectory execution succeeded!")
                response.success = True
                response.message = "Trajectory execution succeeded!"
                msg.data = "Trajectory execution completed successfully!"
            else:
                err_code = result.result.error_code
                self.get_logger().error(f"Trajectory execution failed with error code: {err_code}")
                response.success = False
                response.message = f"Trajectory execution failed with error code: {err_code}"
                msg.data = f"Trajectory execution failed with error code: {err_code}"

            # Publish message when execution is done
            self.trajectory_done_publisher.publish(msg)
            self.get_logger().info("publishing msg")

        except Exception as e:
            self.get_logger().error(f"Trajectory execution result callback failed: {e}")
            response.success = False
            response.message = "Trajectory execution result callback failed."
            msg = String()
            msg.data = "Trajectory execution result callback failed."
            self.trajectory_done_publisher.publish(msg)

    def handle_move_a_to_b(self, request, response):
        """
        Service callback for 'move_a_to_b' that:
          1) Calls 'compute_path' asynchronously
          2) Once completed, calls 'compute_trajectory' asynchronously
          3) Once completed, sends trajectory to FollowJointTrajectory action server asynchronously
          4) Finally, fills the response with success/failure
        """
        self.get_logger().info(f"Received move_a_to_b request:\nStart: {request.pose_start}\nEnd: {request.pose_end}")

        # Prepare request for compute_path
        path_req = ComputePath.Request()
        path_req.pose_start = request.pose_start
        path_req.pose_end = request.pose_end
        path_req.num_interpolations = 4  # Adjust as needed

        self.get_logger().info("Calling compute_path service...")
        future_path = self.compute_path_client.call_async(path_req)
        future_path.add_done_callback(lambda future: self.handle_compute_path_response(future, response))

        return response  # Service completes when `response` is set asynchronously

    def handle_compute_path_response(self, future, response):
        """Handles response from compute_path service and calls compute_trajectory service."""
        try:
            path_res = future.result()
            self.get_logger().info(f"Received path with {len(path_res.poses)} poses from compute_path.")

            # Prepare request for compute_trajectory
            pose_array = PoseArray()
            pose_array.poses = path_res.poses
            traj_req = ComputeTrajectory.Request()
            traj_req.array = pose_array

            self.get_logger().info("Calling compute_trajectory service...")
            future_traj = self.compute_trajectory_client.call_async(traj_req)
            future_traj.add_done_callback(lambda future: self.handle_compute_trajectory_response(future, response))

        except Exception as e:
            self.get_logger().error(f"compute_path service call failed: {e}")
            response.success = False
            response.message = "Failed to compute path."

    def handle_compute_trajectory_response(self, future, response):
        """Handles response from compute_trajectory service and sends trajectory to action server."""
        try:
            traj_res = future.result()
            self.get_logger().info(f"Received trajectory with {len(traj_res.trajectory.points)} points.")

            if not traj_res.trajectory.joint_names or not traj_res.trajectory.points:
                self.get_logger().error("Received an empty trajectory! Exiting...")
                response.success = False
                response.message = "Received an empty trajectory."
                return

            # Send trajectory to action server
            self.send_action_goal(traj_res.trajectory, response)

        except Exception as e:
            self.get_logger().error(f"compute_trajectory service call failed: {e}")
            response.success = False
            response.message = "Failed to compute trajectory."

    def send_action_goal(self, trajectory, response):
        """Sends the computed trajectory to the action server."""
        self.get_logger().info("Sending trajectory to action server...")

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, response))

    def goal_response_callback(self, future, response):
        """Handles response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory execution was rejected by the controller.")
            response.success = False
            response.message = "Trajectory execution was rejected."
            return

        self.get_logger().info("Trajectory execution accepted, waiting for result...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(lambda future: self.result_callback(future, response))



def main(args=None):
    rclpy.init(args=args)

    # Use MultiThreadedExecutor to avoid deadlocks when blocking on futures
    executor = MultiThreadedExecutor()
    node = MoveAToBServer()
    executor.add_node(node)

    try:
        node.get_logger().info("move_a_to_b server is up. Waiting for service requests...")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

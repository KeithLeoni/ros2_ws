import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    script_path = os.path.join(
        os.getcwd(), 'src', 'pose_estimator_pkg', 'launch', 'start_pose_estimator.sh'
    )

    return LaunchDescription([
        # Run the shell script to start the original node
        ExecuteProcess(
            cmd=[script_path],
            shell=True,
            output='screen'
        ),

        # Add the icp_pose_estimator_node node
        Node(
            package='pose_estimator_pkg',
            executable='pose_estimator_node',
            name='pose_estimator_node',
            output='screen'
        ),
        
        # Add the detection_result_republisher node
        Node(
            package='pose_estimator_pkg',
            executable='detection_result_republisher',
            name='detection_result_republisher',
            output='screen'
        ),
        
    ])

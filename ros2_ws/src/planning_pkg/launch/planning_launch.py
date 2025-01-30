import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Add the icp_pose_estimator_node node
        Node(
            package='planning_pkg',
            executable='move_a_to_b_server',
            name='move_a_to_b_server',
            output='screen'
        ),
        
        # Add the detection_result_republisher node
        Node(
            package='planning_pkg',
            executable='block_frame_publisher',
            name='block_frame_publisher',
            output='screen'
        ),
    ])


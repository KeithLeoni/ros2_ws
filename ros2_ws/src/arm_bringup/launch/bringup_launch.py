import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  

def generate_launch_description():
    # Paths to the launch files
    ros2_ur5_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros2_ur5_interface"), "/launch/sim.launch.py"]
        )
    )

    pose_estimator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("pose_estimator_pkg"), "/launch/vision_launch.py"]
        )
    )

    motion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("motion_pkg"), "/launch/motion_launch.py"]
        )
    )

    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("planning_pkg"), "/launch/planning_launch.py"]
        )
    )

    table_avoidance_node = Node(
        package='collision_detection_pkg',
        executable='table_avoidance_node',
        name='table_avoidance_node',
        output='screen',  # Change to 'screen' if you want to see real-time output
        # prefix="bash -c 'exec > /dev/null 2>&1 && '"  # Uncomment to suppress output
    )

    return launch.LaunchDescription([
        ros2_ur5_interface_launch,
        pose_estimator_launch,
        motion_launch,
        planning_launch,
        table_avoidance_node
    ])


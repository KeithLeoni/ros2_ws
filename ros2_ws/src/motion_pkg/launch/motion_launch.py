import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='motion_pkg',
            executable='inverse_kin_server_node',
            name='inverse_kin_server',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        launch_ros.actions.Node(
            package='motion_pkg',
            executable='direct_kin_server',
            name='direct_kin_server',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        launch_ros.actions.Node(
            package='motion_pkg',
            executable='compute_trajectory_service',
            name='compute_trajectory_service',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        launch_ros.actions.Node(
            package='motion_pkg',
            executable='compute_path_service',
            name='compute_path_service',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
    ])
